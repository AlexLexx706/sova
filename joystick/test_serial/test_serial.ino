/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-joystick
 */

#define VRX_PIN  34 // ESP32 pin GPIO36 (ADC0) connected to VRX pin
#define VRY_PIN  35 // ESP32 pin GPIO39 (ADC0) connected to VRY pin
#define BUTTON_PIN  32 // JOYSTIC button

int valueX = 0; // to store the X-axis value
int valueY = 0; // to store the Y-axis value

void setup() {
  Serial.begin(115200) ;
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  // read X and Y analog values
  valueX = analogRead(VRX_PIN);
  valueY = analogRead(VRY_PIN);

  // print data to Serial Monitor on Arduino IDE
  Serial.print("x = ");
  Serial.print(valueX);
  Serial.print(", y = ");
  Serial.print(valueY);
  Serial.print("btn:");
  Serial.println(digitalRead(BUTTON_PIN));
  delay(200);
}
