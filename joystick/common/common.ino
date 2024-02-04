#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Keypad.h>

//#define PRINT_DEBUG

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // three columns
#define FREQUENCY 20

#define VRX_PIN  34 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRY_PIN  35 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
#define BUTTON_PIN  32 // JOYSTIC button

int value_x = 0; // to store the X-axis value
int value_y = 0; // to store the Y-axis value


char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_column[COLUMN_NUM] = {19, 18, 5, 17}; // GPIO19, GPIO18, GPIO5, GPIO17 connect to the row pins
byte pin_rows[ROW_NUM] = {16, 4, 0, 2};   // GPIO16, GPIO4, GPIO0 connect to the column pins

Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

// REPLACE WITH THE MAC Address of your receiver 
uint8_t rev_1_address[] = {0x08, 0xD1, 0xF9, 0x99, 0x3F, 0x9C};
uint8_t rev_2_address[] = {0xA8, 0x42, 0xE3, 0xCE, 0x2C, 0xD8};


esp_now_peer_info_t peerInfo;
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register peer rev-2
  memcpy(peerInfo.peer_addr, rev_2_address, sizeof(rev_2_address));
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

struct JoystickState{
  char protocol = 0;
  char row[4];
  float j_left_right;
  float j_front_back;
  char j_b;
} joystick_state;

void loop() {
  // update keyboard
  keypad.getKeys();

  //read joystick button
  joystick_state.j_b = digitalRead(BUTTON_PIN);

  // read X and Y analog values
  value_x = analogRead(VRX_PIN);
  value_y = analogRead(VRY_PIN);

  joystick_state.row[0] = keypad.bitMap[0];
  joystick_state.row[1] = keypad.bitMap[1];
  joystick_state.row[2] = keypad.bitMap[2];
  joystick_state.row[3] = keypad.bitMap[3];
  
  joystick_state.j_left_right = value_y > 1870 ? (1870. - value_y) / (4095 - 1870.) :  (1870 - value_y) / 1870.;
  joystick_state.j_front_back = value_x < 1870 ? (1870. - value_x) / 1870.:  (1870 - value_x) / (4095 - 1870.);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(0, (const uint8_t*)(&joystick_state), sizeof(joystick_state));

  delay(1000/FREQUENCY);

//debug
#ifdef PRINT_DEBUG
  Serial.print("row 1:");
  Serial.println(keypad.bitMap[0], BIN);

  Serial.print("row 2:");
  Serial.println(keypad.bitMap[1], BIN);

  Serial.print("row 3:");
  Serial.println(keypad.bitMap[2], BIN);

  Serial.print("row 4:");
  Serial.println(keypad.bitMap[3], BIN);

  // print data to Serial Monitor on Arduino IDE
  Serial.print("x = ");
  Serial.println(value_x);

  Serial.print(", y = ");
  Serial.println(value_y);

  Serial.print("btn:");
  Serial.println(digitalRead(BUTTON_PIN));

  //y: left-midle-right: 4095--1870--0
  Serial.print("lr:");
  Serial.println(joystick_state.j_left_right);

  //x: front-midle-back: 0--1870--4095 
  Serial.print("fb:");
  Serial.println(joystick_state.j_front_back);
 
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
#endif
}
