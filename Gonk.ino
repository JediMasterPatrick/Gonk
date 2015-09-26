/*
 * Patrick's project...
 */


// ** Change this, run tests 14 through 20
#define TEST_PIN 14
 
// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// These are the pins used for the breakout example
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

#define LED_PIN 13

struct Track {
    uint8_t vol;
    char *mp3;
};

struct InputPin {
  uint8_t pin;
  uint8_t vol;
  unsigned long timeOn;
  struct Track *tracks;
  uint8_t nTracks;
};

// { vol, track}
struct Track Tracks[] = { 
                   { 10, "tr1.mp3" },
                   { 10, "tr2.mp3" },
                   { 10, "tr3.mp3" },
                   { 10, "tr4.mp3" },
                   { 10, "tr5.mp3" },
                   { 10, "tr6.mp3" },
                   { 10, "tr7.mp3" },
                   { 10, "tr8.mp3" },
                   { 10, "tr9.mp3" },
                   { 10, "tr10.mp3" },
                   { 10, "tr11.mp3" },
                   { 10, "tr12.mp3" },
                   { 10, "tr13.mp3" },
};
uint8_t nTracks = sizeof(Tracks)/sizeof(*Tracks);

//attack waves
//struct Track Pin1Tracks[] = { 
//                   { 10, "a1.wav" },
//                   { 10, "a2.wav" },
//                   { 10, "a3.wav" },
//                   { 10, "a4.wav" },
//                   { 10, "a5.wav" },
//                   { 10, "a6.wav" },
//                   { 10, "a7.wav" },
//};
//beep waves
struct Track Pin2Tracks[] = { 
                   { 10, "b1.ogg" },
                   { 10, "b2.ogg" },
                   { 10, "b3.wav" },
                   { 10, "b4.wav" },
                   { 10, "b5.wav" },
                   { 10, "b6.wav" },
                   { 10, "b7.wav" },
                   { 10, "b8.wav" },
                   { 10, "b9.ogg" },
                   { 10, "b10.ogg" },
                   { 10, "b11.mp3" },
                   { 10, "b12.mp3" },  
                   { 10, "b13.wav" },
};

//chatter waves
struct Track Pin5Tracks[] = { 
                   { 10, "c1.ogg" },
                   { 10, "c2.wav" },
                   { 10, "c3.ogg" },
                   { 10, "c4.wav" },
                   { 10, "c5.wav" },//test
                   { 10, "c6.wav" },
                   { 10, "c7.wav" },
                   { 10, "c8.wav" },
                   { 10, "c9.wav" },
                   { 10, "c10.ogg" },
                   { 10, "c11.ogg" },
                   { 10, "c12.ogg" },
                   { 10, "c13.wav" },
                   { 10, "c14.ogg" },
                   { 10, "c15.wav" },
};

//sound waves, chewy waves
//struct Track Pin12Tracks[] = { 
//                   { 10, "s1.ogg" },
//                   { 10, "s2.wav" },
//                   { 10, "s3.ogg" },
//                   { 10, "s4.ogg" },
//                   { 10, "ch1.wav" },
//                   { 10, "ch2.wav" },
//                   { 10, "ch3.wav" },
//                   { 10, "ch4.wav" },
//};
//scream waves
struct Track Pin12Tracks[] = { 
                     { 10, "1.wav" },
                     { 10, "2.wav" },
                     { 10, "3.ogg" },               
                     { 10, "4.wav" },
                     { 10, "5.ogg" },
};



//uint8_t nPin1Tracks = sizeof(Pin1Tracks)/sizeof(*Pin1Tracks);
uint8_t nPin2Tracks = sizeof(Pin2Tracks)/sizeof(*Pin2Tracks);
uint8_t nPin5Tracks = sizeof(Pin5Tracks)/sizeof(*Pin5Tracks);
//uint8_t nPin11Tracks = sizeof(Pin11Tracks)/sizeof(*Pin11Tracks);
uint8_t nPin12Tracks = sizeof(Pin12Tracks)/sizeof(*Pin12Tracks);

// Pin IDs, these are the input pins (blank �� == play/pause)
// { pin, vol, 0, �track name.mp3� }
struct InputPin Inputs[] = { { 0, 0, 0, 0, 0 },
                            // { 1, 10, 0, Pin1Tracks, nPin1Tracks },
                             { 2, 10, 0, Pin2Tracks, nPin2Tracks },
                             { 5, 10, 0, Pin5Tracks, nPin5Tracks },
                            // { 11, 10, 0, Pin11Tracks, nPin11Tracks },
                             { 12, 10, 0, Pin12Tracks, nPin12Tracks },
                             { TEST_PIN, 10, 0, Pin12Tracks, nPin12Tracks },
                           };                                                                                                                                                          
uint8_t nInputs = sizeof(Inputs)/sizeof(*Inputs);
uint8_t Play = 0;

// music player
Adafruit_VS1053_FilePlayer Music = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);
//Adafruit_VS1053_FilePlayer gMusic = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

void exit() {
  while(1);
}

void stopMusic() {
  Music.stopPlaying();
  Music.softReset();
}

void playMusic(char *mp3,uint8_t vol) {
  stopMusic();
  Music.setVolume(vol,vol);
  if(!Music.startPlayingFile(mp3)) {
    Serial.print("Could not open ");
    Serial.println(mp3);
    exit();
  }
  Serial.print("Playing: ");
  Serial.println(mp3);
}

void setup() {
  // Init the serial port
  Serial.begin(9600);
  
  // initialise the music player
  if(!Music.begin()) {
     Serial.println(F("No VS1053"));
     exit();
  }

  
  Music.setVolume(10,10);
  for(uint8_t pin=0;pin<nInputs;++pin) {
      pinMode((Inputs+pin)->pin,INPUT_PULLUP);
  }
  
pinMode(LED_PIN, OUTPUT);

if (! Music.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
   Serial.println(F("DREQ not int")); 

if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed"));
    exit();
  }

}

void loop() {
  // 
  // current millisecond
  unsigned long now=millis();


//this is the gpiotest
//  for (uint8_t i=0; i<8; i++) { 
//    gMusic.GPIO_pinMode(i, OUTPUT);
//    
//    gMusic.GPIO_digitalWrite(i, HIGH);
//    Serial.print("GPIO: "); Serial.println(gMusic.GPIO_digitalRead(i));
//    gMusic.GPIO_digitalWrite(i, LOW);
//    Serial.print("GPIO: "); Serial.println(gMusic.GPIO_digitalRead(i));
//
//    gMusic.GPIO_pinMode(i, INPUT);
//
//    delay(100);  
//  }

//this is the end of the gpiotest
  
  // read the pins
  for(uint8_t pin=0;pin<nInputs;++pin) {
    // read the pin
    if(digitalRead((Inputs+pin)->pin)) {
      // make sure we haven't been set on before
      if(!(Inputs+pin)->timeOn) {
        // set this pin to on!
        (Inputs+pin)->timeOn=now;
        // turn on LED
        digitalWrite(LED_PIN,HIGH);
        if((Inputs+pin)->tracks) {
          // Play random track
          uint8_t tk=random((Inputs+pin)->nTracks);
          Track *track=(Inputs+pin)->tracks+tk;
          // play track
          playMusic(track->mp3,track->vol);
         }
         else {
             Play=(Play+1)&1;
             if(!Play) {
               stopMusic();
           }
         }
      }
    }
    else if((Inputs+pin)->timeOn) {
      // make sure pin state held for 10ms (handle switch bounce)
      if((Inputs+pin)->timeOn+10<now) {
        // reset this pin
        (Inputs+pin)->timeOn=0;
        // turn off LED
        digitalWrite(LED_PIN,LOW);
      }
    }
  }
  // playing anything?
  if(Play&&!Music.playingMusic) {
    // no, start something
    delay(200); //200ms delay
    int r=random(nTracks);
    playMusic((Tracks+r)->mp3,(Tracks+r)->vol);
  }
  
}


