#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

#define PRINT_FOR_SERIAL_PLOTTER
// #define PRINT_NO_LIM
// #define PRINT_WITH_LIM
// #define PRINT_VALUE
// #define PRINT_ESP_NOW_ERROR

// Potentiometers desc
struct PotDesc {
    int pin;
    int min;
    int max;
    int len;
    char *name;
    int row_no_lim;
    int row_lim;
    float val;
};

PotDesc descs[] = {
    {34, 700, 2350, 2350 - 700, "head"},     // head
    {35, 290, 2320, 2320 - 290, "neak"},     // neak
    {32, 1450, 3000, 3000 - 1450, "l_wing"}, // left wing
    {33, 1100, 2600, 2600 - 1100, "r_wing"}, // right wing
    {39, 600, 3600, 3600 - 600, "body"}      // body
};

// REPLACE WITH THE MAC Address of your receiver
uint8_t rev_1_address[] = {0x08, 0xD1, 0xF9, 0x99, 0x3F, 0x9C};
uint8_t rev_2_address[] = {0xB0, 0xB2, 0x1C, 0x0A, 0x07, 0x08};
esp_now_peer_info_t peerInfo;

void setup() {
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

    // Register peer rev-1
    memcpy(peerInfo.peer_addr, rev_1_address, sizeof(rev_1_address));

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Register peer rev-2
    memcpy(peerInfo.peer_addr, rev_2_address, sizeof(rev_2_address));

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // set the resolution to 12 bits (0-4096)
    analogReadResolution(12);

    delay(1000);
}

struct JoystickState {
    char protocol = 1;
    float vals[5] = {0};
} joystick_state;

void loop() {
    // Reading potentiometer value
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc &cur_desc = descs[i];
        cur_desc.row_no_lim = analogRead(cur_desc.pin);
        cur_desc.row_lim = cur_desc.row_no_lim;

        if (cur_desc.row_lim < cur_desc.min) {
            cur_desc.row_lim = cur_desc.min;
        } else if (cur_desc.row_lim > cur_desc.max) {
            cur_desc.row_lim = cur_desc.max;
        }

        cur_desc.val = (cur_desc.row_lim - cur_desc.min) / float(cur_desc.len);
        joystick_state.vals[i] = cur_desc.val;
    }

// Send message via ESP-NOW
// display send result
#ifdef PRINT_ESP_NOW_ERROR
    esp_err_t result = esp_now_send(0, (const uint8_t *)(&joystick_state),
                                    sizeof(joystick_state));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
#else
    esp_now_send(0, (const uint8_t *)(&joystick_state), sizeof(joystick_state));
#endif

// display potentiometers debug
#ifdef PRINT_NO_LIM
    Serial.print("No lim: ");
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc &cur_desc = descs[i];
        Serial.print(cur_desc.name);
        Serial.print(": ");
        Serial.print(cur_desc.row_no_lim);
        Serial.print(" ");
    }
    Serial.println();
#endif

#ifdef PRINT_FOR_SERIAL_PLOTTER
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc &cur_desc = descs[i];
        Serial.print(cur_desc.row_no_lim);
        Serial.print(" ");
    }
    Serial.println();
#endif

#ifdef PRINT_WITH_LIM
    Serial.print("with lim: ");
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc &cur_desc = descs[i];
        Serial.print(cur_desc.name);
        Serial.print(": ");
        Serial.print(cur_desc.row_lim);
        Serial.print(" ");
    }
    Serial.println();
#endif

#ifdef PRINT_VALUE
    Serial.print("value: ");
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc &cur_desc = descs[i];
        Serial.print(cur_desc.name);
        Serial.print(": ");
        Serial.print(cur_desc.val);
        Serial.print(" ");
    }
    Serial.println();
#endif

    delay(20);
}
