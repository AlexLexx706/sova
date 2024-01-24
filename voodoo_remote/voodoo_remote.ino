
//#define PRINT_NO_LIM
//#define PRINT_WITH_LIM
//#define PRINT_VALUE

// Potentiometers desc
struct PotDesc {
  int pin;
  int min;
  int max;
  int len;
  char * name;
  int row_no_lim;
  int row_lim;
  float val;
};

PotDesc descs[] = {
  {34, 700, 2350, 2350 - 700, "head"},    //head
  {35, 290, 2320, 2320 - 290,  "neak"},    //neak
  {32, 1450, 3000, 3000 - 1450, "l_wing"}, //left wing
  {33, 1100, 2600, 2600 - 1100,  "r_wing"}, //right wing
  {25, 600, 3600, 3600 - 600, "body"}      //body
};

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
      PotDesc & cur_desc = descs[i];
      cur_desc.row_no_lim = analogRead(cur_desc.pin);
      cur_desc.row_lim = cur_desc.row_no_lim;

      if (cur_desc.row_lim < cur_desc.min) {
          cur_desc.row_lim = cur_desc.min;
      } else if (cur_desc.row_lim > cur_desc.max) {
          cur_desc.row_lim = cur_desc.max;
      }

      cur_desc.val = (cur_desc.row_lim - cur_desc.min) / float(cur_desc.len);
  }

  #ifdef PRINT_NO_LIM
    Serial.print("No lim: ");
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc & cur_desc = descs[i];
      Serial.print(cur_desc.name);
      Serial.print(": ");
      Serial.print(cur_desc.row_no_lim);
      Serial.print(" ");
    }
    Serial.println();
  #endif

  #ifdef PRINT_WITH_LIM
    Serial.print("with lim: ");
    for (int i = 0; i < sizeof(descs) / sizeof(descs[0]); i++) {
        PotDesc & cur_desc = descs[i];
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
        PotDesc & cur_desc = descs[i];
      Serial.print(cur_desc.name);
      Serial.print(": ");
      Serial.print(cur_desc.val);
      Serial.print(" ");
    }
    Serial.println();
  #endif
  
  delay(100);
}
