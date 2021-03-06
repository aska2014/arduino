#include "Arduino.h"
#include "SoftwareSerial.h"
#include "EasyVR.h"

#define SND_Access_denied            1
#define SND_Access_granted           2
#define SND_Hello                    3
#define SND_Please_repeat            4
#define SND_Please_say_your_password 6
#define SND_Please_talk_louder       7


SoftwareSerial port(12,13);

EasyVR easyvr(port);
EasyVRBridge bridge;
uint32_t mask = 0;
int8_t group = 0;
uint8_t train = 0;
char name[32];
int count=0;
boolean passarea=false;

int buzzerPin=3;
int motorPin=4;

void setup() {
    // bridge mode?
    if (bridge.check()) {
        cli();
        bridge.loop(0, 1, 12, 13);
    }
    Serial.begin(9600);
    port.begin(9600);
    if (!easyvr.detect()) {
        Serial.println("EasyVR not detected!");
        for (;;);
    }
    easyvr.setPinOutput(EasyVR::IO1, LOW);
    Serial.println("EasyVR detected!");
    easyvr.setTimeout(5);
    easyvr.setLanguage(EasyVR::ITALIAN);
    int16_t count = 0;
    if (easyvr.getGroupMask(mask)){ // get trained user names and passwords 
        uint32_t msk = mask;
        for (group = 0; group <= EasyVR::PASSWORD; ++group, msk >>= 1) {
            if (!(msk & 1)) continue;
            if (group == EasyVR::TRIGGER)
                    Serial.print("Trigger: "); else if (group == EasyVR::PASSWORD)
                    Serial.print("Password: "); else {
                Serial.print("Group ");
                Serial.print(group);
                Serial.print(": ");
            }
            count = easyvr.getCommandCount(group);
            Serial.println(count);
            for (int8_t idx = 0; idx < count; ++idx) {
                if (easyvr.dumpCommand(group, idx, name, train)) {
                    Serial.print(idx);
                    Serial.print(" = ");
                    Serial.print(name);
                    Serial.print(", Trained ");
                    Serial.print(train, DEC);
                    if (!easyvr.isConflict())
                                Serial.println(" times, OK"); else {
                        int8_t confl = easyvr.getWord();
                        if (confl >= 0)
                                      Serial.print(" times, Similar to Word "); else {
                            confl = easyvr.getCommand();
                            Serial.print(" times, Similar to Command ");
                        }
                        Serial.println(confl);
                    }
                }
            }
        }
    }
    easyvr.setLevel(EasyVR::HARDER);
    easyvr.playSound(SND_Hello, EasyVR::VOL_FULL);


    // Setup pins
    pinMode(buzzerPin, OUTPUT);
    pinMode(motorPin, OUTPUT);

}



void loop() {
    int idx_cmd;
    int idx_pwd;
    easyvr.setPinOutput(EasyVR::IO1, HIGH);
    // LED on (listening)
    passarea=false;
    Serial.println("Say a name in Group 1");
    easyvr.recognizeCommand(1);
    // recognise command in group 1 
    while (!easyvr.hasFinished());
    // wait for user name
    easyvr.setPinOutput(EasyVR::IO1, LOW);
    // LED off
    idx_cmd = easyvr.getCommand();
    // get recognised user name
    if (idx_cmd >= 0) {
        Serial.print("Name: ");
        if (easyvr.dumpCommand(1, idx_cmd, name, train))
              Serial.println(name); else
              Serial.println();
        easyvr.playSound(SND_Please_say_your_password , EasyVR::VOL_FULL);
        // ask for password
        passarea=true;
        easyvr.setPinOutput(EasyVR::IO1, HIGH);
        // LED on (listening)  
        Serial.println("Say the password");
        easyvr.recognizeCommand(EasyVR::PASSWORD);
        // set group 16
        while (!easyvr.hasFinished());
        // wait for password
        easyvr.setPinOutput(EasyVR::IO1, LOW);
        // LED off    
        idx_pwd = easyvr.getCommand();
        // get recognised password
        if (idx_pwd >= 0) {
            Serial.print("Password: ");
            if (easyvr.dumpCommand(EasyVR::PASSWORD, idx_pwd, name, train)) {
                Serial.print(" = ");
                Serial.println(name);
            } else
                    Serial.println();
            if ( idx_pwd == idx_cmd){ // index of username and password are the same, access granted 
                count=0;
                Serial.println("Access granted");
                easyvr.playSound(SND_Access_granted , EasyVR::VOL_FULL);
                while (true) {
                    easyvr.setPinOutput(EasyVR::IO1, HIGH);
                    Serial.println("Say a command in Group 2");
                    easyvr.recognizeCommand(2);
                    // recognise command in group 1 
                    while (!easyvr.hasFinished());
                    // wait for user name
                    easyvr.setPinOutput(EasyVR::IO1, LOW);
                    // LED off
                    idx_cmd = easyvr.getCommand();
                    // get recognised user name
                    if (idx_cmd >= 0) {
                        Serial.print("Name: ");
                        if (easyvr.dumpCommand(1, idx_cmd, name, train))
                              Serial.println(name); else
                              Serial.println();
                        if (idx_cmd == 0) {
                            startMotor();
                            easyvr.playSound(SND_Access_granted , EasyVR::VOL_FULL);
                        }
                        if (idx_cmd == 1) {
                            easyvr.playSound(SND_Access_granted , EasyVR::VOL_FULL);
                            stopMotor();
                            break;
                        }
                    }
                }
            } else // index of username and password differ, access is denied 
            {
                Serial.println("Access denied");
                easyvr.playSound(SND_Access_denied , EasyVR::VOL_FULL);
            }
        }
    }
    int16_t err = easyvr.getError();
    
    if (easyvr.isTimeout() || (err >= 0)) // password timeout, access is denied 
       {
        Serial.println("Error, try again...");
        if (passarea) {
            count++;
            easyvr.playSound(SND_Access_denied , EasyVR::VOL_FULL);
        }
        Serial.println(" ///////////////////////////////////////////////////////////////////");
        Serial.println(count);
        Serial.println(" ///////////////////////////////////////////////////////////////////");
        if (count==3) {

            // Open buzzer
            openBuzzer();
            while(1) {
            }
        }
    } else {
        if (easyvr.isTimeout()) 
                Serial.println("Timed out, try again...");
        int16_t err = easyvr.getError();
        if (err >= 0) {
            Serial.print("Error ");
            Serial.println(err, HEX);
        }
    }
}







void openBuzzer()
{
  analogWrite(9, HIGH);
}


void startMotor()
{
    digitalWrite(10, HIGH);
}

void stopMotor()
{
    digitalWrite(10, LOW);
}
