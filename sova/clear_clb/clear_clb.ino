#include <EEPROM.h>

struct ServoClbInfo {
    struct Point {
        float row = 0;
        float angle = 0;
    };
    Point min;
    Point max;
    bool calibrated = false;
};

ServoClbInfo servo_clb_data[5];

void setup() {
    Serial.begin(115200);

    //reading calibration 
    EEPROM.begin(sizeof(servo_clb_data));
    EEPROM.writeBytes(0, servo_clb_data, sizeof(servo_clb_data));
    EEPROM.commit();
    Serial.println("calibration cleared");
}


void loop() {
    delay(10);
}
