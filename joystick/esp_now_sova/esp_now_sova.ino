#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// REPLACE WITH THE MAC Address of your receiver 

uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0xE5, 0x6D, 0xA0};

esp_now_peer_info_t peerInfo;

struct JoystickState{
  char row[4];
  float j_left_right;
  float j_front_back;
  char j_b;
} joystick_state;



// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incoming_data, int len) {
  if (len == sizeof(JoystickState)) {
    joystick_state = *(JoystickState *)(incoming_data);

    Serial.print("row 1:");
    Serial.println(joystick_state.row[0], BIN);
  
    Serial.print("row 2:");
    Serial.println(joystick_state.row[1], BIN);
  
    Serial.print("row 3:");
    Serial.println(joystick_state.row[2], BIN);
  
    Serial.print("row 4:");
    Serial.println(joystick_state.row[3], BIN);

    Serial.print("lr:");
    Serial.println(joystick_state.j_left_right);

    Serial.print("fb:");
    Serial.println(joystick_state.j_front_back);  
  } else {
    Serial.print("Bytes received: ");
    Serial.println(len);
  }
}
 
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

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

const char * msg = "hi";

void loop() {
  delay(10000);
}
