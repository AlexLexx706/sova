#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
DFRobotDFPlayerMini myDFPlayer;


int action = 0;
int Song = 0;

void setup() {
 Serial2.begin(9600);
 Serial.begin(115200);
  if (!myDFPlayer.begin(Serial2)) {  //Serial2 to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
       delay(0); // Code to compatible with ESP8266 watch dog.   
  }
    Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  Serial.println("The device started, now you can pair it with bluetooth!");

  myDFPlayer.volume(2);
}

//state: 0 - stop
//state: 1 - play
//state: -1 - unknown (use previous state)

void loop() {
  myDFPlayer.play(1);
  delay(5000);
  myDFPlayer.play(2);
  delay(5000);
  myDFPlayer.play(3);
  int i = 0;
  while (1) {
    Serial.print("Sate:");
    Serial.println(myDFPlayer.readState());
    delay(1000);
  }

  myDFPlayer.stop();
  i = 0;
  while (++i < 5) {
    Serial.print("Sate:");
    Serial.println(myDFPlayer.readState());
    delay(1000);
  }

}
void __loop() {
  action = Serial.read();
  if (action != -1) {
 Serial.println (action);    //Print what we receive 

    switch (action) {
      case 1:
       myDFPlayer.previous();  //Play previous mp3
       break;
       
       case 2:
       myDFPlayer.pause(); //Stop
       Song = 0;
       break;
       
       case 3:  
       if (Song == 0) {
        myDFPlayer.play(1);  //Play from beginning  
        myDFPlayer.enableLoopAll();
        Song = 1;     
       }
       else {
        myDFPlayer.start(); //Play from Pause
        myDFPlayer.disableLoopAll();
       }
       break;
       
       case 4:
       myDFPlayer.pause(); //Pause
       break;
       
       case 5:
       myDFPlayer.next(); //Next
       break;
       
       case 6:
       myDFPlayer.volumeDown(); //Volume +
       break;
       
       case 7:
       myDFPlayer.volumeUp(); //Volume -
       break;

       case 8:
       myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
       break;

       case 9:
       myDFPlayer.EQ(DFPLAYER_EQ_POP);
       break;

       case 10:
       myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
       break;

       case 11:
       myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
       break;

       case 12:
       myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
       break;

       case 13:
       myDFPlayer.EQ(DFPLAYER_EQ_BASS);
       break;

       case 14:
       myDFPlayer.loopFolder(1);
       break;

       case 15:
       myDFPlayer.loopFolder(2);
       break; 
           
       case 16:
       myDFPlayer.loopFolder(3);
       break;
       
       case 17:
       myDFPlayer.loopFolder(4);
       break;
       
       case 18:
       myDFPlayer.loopFolder(5);
       break; 

       case 19:
       myDFPlayer.loopFolder(6);
       break; 

       case 20:
       myDFPlayer.loopFolder(7);
       break;

       case 21:
       myDFPlayer.loopFolder(8);
       break;
       
       case 22:
       myDFPlayer.loopFolder(9);
       break;

       case 23:
       myDFPlayer.loopFolder(10);
       break; 
                      
    }
    
  }
  }
