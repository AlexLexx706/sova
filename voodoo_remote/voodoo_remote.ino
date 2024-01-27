#include "exponential_filter.h"
#include <CommandParser.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

/*commands stream from UART, ascii symbols:
    debug value
        show/hide, int, stream of axis values: 0 - hide(default), 1 - show
    clb_start clb_axis ref_axis min max
        start calibration of axis: 0-head, 1-neck, 2-left hand,
            3-right hand, 4-body
        clb_axis - int, calibrated axis id
        ref_axis - int, reference axis id,
            used for rotating servo in correct position
        min - float, value in axis value (row) of ref axis
            which will be interpreted as 0 deg in servo side
        max - float, value in axis value (row) of ref axis
            which will be interpreted as 180 deg in servo side
    clb_stop cmd
        stop calibration of axis
        cmd - int, command:
            0 - not apply calibration and stop calibration process
            1 - save point 1
            2 - save point 2
            3 - apply calibration and stop calibration process
    reset
        switch to normal mode

*/
typedef CommandParser<> MyCommandParser;

MyCommandParser parser;
TaskHandle_t cmd_task;
SemaphoreHandle_t xMutex = NULL; // Create a mutex object

char line[128];
char response[MyCommandParser::MAX_RESPONSE_SIZE];
bool show_debug = false;

struct ModeState {
    char mode = 0; // 0 - normal mode, 1 - start calibration, 2 - stop calibration
    char clb_axis = 0;
    char ref_axis = 4;
    float min = 496;
    float max = 3293;
} mode_state;

void cmd_debug(MyCommandParser::Argument *args, char *response) {
    show_debug = bool(args[0].asInt64);
    Serial.print("show_debug: ");
    Serial.println(show_debug);
}

void cmd_start_clb(MyCommandParser::Argument *args, char *response) {
    int clb_axis = args[0].asUInt64;
    int ref_axis = args[1].asUInt64;
    float min = args[2].asDouble;
    float max = args[3].asDouble;

    Serial.print("clb_axis: ");
    Serial.print(clb_axis);
    Serial.print(" ref_axis: ");
    Serial.print(ref_axis);
    Serial.print(" min: ");
    Serial.print(min);
    Serial.print(" max: ");
    Serial.println(max);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        mode_state.mode = 1;
        mode_state.clb_axis = clb_axis;
        mode_state.ref_axis = ref_axis;
        mode_state.min = min;
        mode_state.max = max;
        xSemaphoreGive(xMutex);
    }
}

void cmd_stop_clb(MyCommandParser::Argument *args, char *response) {
    int cmd = args[0].asUInt64;
    Serial.print(" cmd: ");
    Serial.println(cmd);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        mode_state.mode = 2;
        mode_state.clb_axis = cmd;
        xSemaphoreGive(xMutex);
    }
}

void cmd_reset_clb(MyCommandParser::Argument *args, char *response) {
    Serial.println("normal mode");

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        mode_state.mode = 0;
        xSemaphoreGive(xMutex);
    }
}

void cmd_test(MyCommandParser::Argument *args, char *response) {
    Serial.print("string: ");
    Serial.println(args[0].asString);
    Serial.print("double: ");
    Serial.println(args[1].asDouble);
    Serial.print("int64: ");
    Serial.println(
        (int32_t)args[2].asInt64); // NOTE: on older AVR-based boards, Serial
                                   // doesn't support printing 64-bit values, so
                                   // we'll cast it down to 32-bit
    Serial.print("uint64: ");
    Serial.println(
        (uint32_t)args[3].asUInt64); // NOTE: on older AVR-based boards, Serial
                                     // doesn't support printing 64-bit values,
                                     // so we'll cast it down to 32-bit
}

void process_cmd(void *_) {
    for (;;) {
        if (Serial.available()) {
            size_t lineLength =
                Serial.readBytesUntil('\n', line, sizeof(line) - 1);
            line[lineLength] = '\0';
            parser.processCommand(line, response);
        } else {
            delay(20);
        }
    }
}

// Potentiometers desc
class PotDesc {
    int pin;
    int row;
    ExponentialFilter<float> exp_filter;

 public:
    PotDesc(int _pin) : pin(_pin), row(0), exp_filter(10., 0.) {}

    float read() {
        row = analogRead(pin);
        return exp_filter.Filter(row);
    }
};

PotDesc desc[] = {
    {34}, // head
    {35}, // neck
    {32}, // left wing
    {33}, // right wing
    {39}  // body
};

// REPLACE WITH THE MAC Address of your receiver
uint8_t rev_2_address[] = {0xB0, 0xB2, 0x1C, 0x0A, 0x07, 0x08};
esp_now_peer_info_t peerInfo;

void setup() {
    xMutex = xSemaphoreCreateMutex(); // crete a mutex object

    Serial.begin(115200);

    // // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Register peer rev-2
    memcpy(peerInfo.peer_addr, rev_2_address, sizeof(rev_2_address));

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    parser.registerCommand("TEST", "sdiu", &cmd_test);
    parser.registerCommand("debug", "i", &cmd_debug);
    parser.registerCommand("clb_start", "uudd", &cmd_start_clb);
    parser.registerCommand("clb_stop", "u", &cmd_stop_clb);
    parser.registerCommand("reset", "", &cmd_reset_clb);

    xTaskCreatePinnedToCore(
        process_cmd, /* Task function. */
        "cmd",       /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        &cmd_task,   /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */

    delay(1000);
}

struct JoystickState {
    char protocol = 1;
    float vals[5] = {0};
    ModeState mode;
} joystick_state;

void loop() {
    // Reading potentiometer value
    for (int i = 0; i < sizeof(desc) / sizeof(desc[0]); i++) {
        joystick_state.vals[i] = desc[i].read();
    }

    // update current state
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        joystick_state.mode = mode_state;
        xSemaphoreGive(xMutex);
    }

    // Send message via ESP-NOW
    esp_now_send(0, (const uint8_t *)(&joystick_state), sizeof(joystick_state));

    if (show_debug) {
        for (int i = 0; i < sizeof(desc) / sizeof(desc[0]); i++) {
            Serial.print(joystick_state.vals[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    delay(20);
}
