/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-keypad-lcd
 */

#include <Keypad.h>

#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // three columns

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_column[COLUMN_NUM] = {19, 18, 5, 17}; // GPIO19, GPIO18, GPIO5, GPIO17 connect to the row pins
byte pin_rows[ROW_NUM] = {16, 4, 0, 2};   // GPIO16, GPIO4, GPIO0 connect to the column pins

Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

void setup(){
  Serial.begin(115200) ;

}

void loop(){
  char key = keypad.getKey();
  Serial.print("btn:");
  Serial.println(key);
  delay(200);
}
