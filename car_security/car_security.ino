#include <String.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_VC0706.h>
#include <SD.h>


/////////////////////////////////////////////////////////////
// DEFINING PINS
/////////////////////////////////////////////////////////////
// CAMERA VARIABLES
SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

// GSM VARIABLES
SoftwareSerial gsmSerial(16, 17);

// GPS VARIABLES
SoftwareSerial gpsSerial(10, 5); // create gps sensor connection
TinyGPS gps; // create gps object
long lat,lon; // create variable for latitude and longitude object
 
// BUZZER PIN
int buzzerPin = 4;

// BUTTONS PINS
int helpBtnPin = 36;
int stopBtnPin = 32;

// SESNOR PIN
int pingPin = 34;

// SD CARD PIN
int sdCardPin = 8;
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////












/////////////////////////////////////////////////////////////
// APPLICATION VARIBABES
/////////////////////////////////////////////////////////////
unsigned long sensorCutAt = 0; // milliseconds the sensor cut at
unsigned int sensorWaitMillis = 30 * 1000; // Number of seconds the sensor has to wait before starting the security system

// Needed variables to send message every one minute
unsigned int sendSmsInterval = 60 * 1000; // send sms interval milliseconds 
String numbersToSend[] = {"", ""}; // Registered numbers to send sms to (max number of registered numbers is 2)
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////


// Dynamic information
String ownerNumber  = "01201109095";
String policeNumber = "01067179264";












void setup()
{
  Serial.begin(19200);    // the GPRS baud rate 
  while(!Serial){
    ;
  }
  
  gpsSerial.begin(19200); // connect gps sensor
  gsmSerial.begin(19200); // the GPRS baud rate

  // Define inputs
  pinMode(helpBtnPin, INPUT);
  pinMode(stopBtnPin, INPUT);

  // Define outputs
  pinMode(buzzerPin, OUTPUT);

  // pinMode(10, OUTPUT); // SS on Uno, etc.

  // see if the card is present and can be initialized:
  if (!SD.begin(sdCardPin)) {
    Serial.println("Card failed, or not present");
    return;
  }  
  
  // Try to locate the camera
  if (! cam.begin()) {
    Serial.println("No camera found?");
    return;
  }

  delay(500);
}

// 
void loop()
{
  // If sensor was cut and the time it has to wait has finished
  if(checkSensor())
  {
    // Reset sensor to start calculating again
    resetSensor();

    // Open buzzer
    openBuzzer();

    // Send text message to owner with gps every (one minute = sendSmsInterval)
    registerNumberToSendSms(ownerNumber);

    // Take picture
    takePicture();
  }

  if(checkHelpBtn())
  {
    openBuzzer();

    registerNumberToSendSms(policeNumber);
  }


  if(checkStopBtn())
  {
    closeBuzzer();

    // Remove all registered nubmers to stop sending sms to them
    removeAllRegisteredNumbers();

    resetSensor();
  }

  // // Check for any message to send
  checkForMessageToSend();
}













/////////////////////////////////////////////////////////////
// Buzzer methods
/////////////////////////////////////////////////////////////
void openBuzzer()
{
  analogWrite(buzzerPin, HIGH);
}

void closeBuzzer()
{
  analogWrite(buzzerPin, LOW);
}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////












/////////////////////////////////////////////////////////////
// BUTTONS methods
/////////////////////////////////////////////////////////////
// Flags
boolean helpBtnFlag = false;
boolean stopBtnFlag = false;
///
boolean checkHelpBtn()
{
  // If help btn is pushed
  if(digitalRead(helpBtnPin) == HIGH && ! helpBtnFlag)
  {
    helpBtnFlag = true;
    return true;
  }

  // Reset help btn flag
  if(digitalRead(helpBtnPin) == LOW)
  {
    helpBtnFlag = false;
  }

  return false;
}

boolean checkStopBtn()
{
  // If help btn is pushed
  if(digitalRead(stopBtnPin) == HIGH && ! stopBtnFlag)
  {
    stopBtnFlag = true;
    return true;
  }

  // Reset help btn flag
  if(digitalRead(stopBtnPin) == LOW)
  {
    stopBtnFlag = false;
  }

  return false;
}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////













/////////////////////////////////////////////////////////////
// GPS methods
/////////////////////////////////////////////////////////////
void loadGpsLocation()
{
  gpsSerial.listen();
  while(gpsSerial.available()){ // check for gps data
   if(gps.encode(gpsSerial.read())){ // encode gps data
    gps.get_position(&lat,&lon); // get latitude and longitude
   }
  }
  Serial.println();
  delay(1000);
}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////













/////////////////////////////////////////////////////////////
// CAMERA methods
/////////////////////////////////////////////////////////////
void takePicture()
{
  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  // cam.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time; (Pointer to the word of the image)
    uint8_t *buffer;
    // bytesToRead = 32 or less
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!

    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("taking picture done!");
  Serial.print(time); Serial.println(" ms elapsed");
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////













/////////////////////////////////////////////////////////////
// SMS methods
/////////////////////////////////////////////////////////////
void registerNumberToSendSms(String number)
{
  // If number already exists then dont register it again
  if(checkNumberAlreadyExits(number)) return;

  // Register number in the next empty slot
  for (int i = 0; i < sizeof(numbersToSend); ++i)
  {
    if(numbersToSend[i].equals(""))
    {
      numbersToSend[i] = number;
    }
  }
}

void removeAllRegisteredNumbers()
{
  // Reset all registered numbers
  for (int i = 0; i < sizeof(numbersToSend); ++i)
  {
    numbersToSend[i] = "";
  }
}


boolean checkNumberAlreadyExits(String number)
{
  // Check if number already exist in the string array
  for (int i = 0; i < sizeof(numbersToSend); ++i)
  {
    if(numbersToSend[i].equals(number))
    {
      return true;
    }
  }

  return false;
}

// Variable to hold previous millis
unsigned long previousMillis = - sendSmsInterval;

void checkForMessageToSend()
{
  if(millis() >= (previousMillis + sendSmsInterval))
  {
    for (int i = 0; i < sizeof(numbersToSend); ++i)
    {
      if(! numbersToSend[i].equals(""))
      {
        sendTextMessageWithGps(numbersToSend[i]);
      }
    }
  }
}

void sendTextMessageWithGps(String number)
{
    loadGpsLocation();
    
    String location = String(lat) + "+" + String(lon);
    
    SendTextMessage(number, location);
}

// this function is to send a sms message
void SendTextMessage(String number, String message)
{
  gsmSerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  gsmSerial.println("AT + CMGS = \"" + number + "\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  gsmSerial.println(message);//the content of the message
  delay(100);
  gsmSerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
  gsmSerial.println();
  Serial.println();
}

// void receiveSms()
// {
//   gsmSerial.listen();
//   while(gsmSerial.available()){ // check for gps data
//    if(gsmSerial.read()){ // encode gps data
   
//    }
//   }
// }
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////












/////////////////////////////////////////////////////////////
// Sensor methods
/////////////////////////////////////////////////////////////
boolean checkSensor()
{
  // If sensor is cut and first time then set sensorCutAt to the current millis
  if(sensorIsTrue() && sensorCutAt == 0)
  {
    sensorCutAt = millis();
  }

  // If sensor greater than zero then it means the sensor has been cut
  // Now return true only if sensor was cut before (30 seconds ago = sensorWaitSeconds)
  if(sensorCutAt > 0)
  {
    // Serial.println("Sensor cut at:" + (sensorCutAt + sensorWaitMillis));
    return (sensorCutAt + sensorWaitMillis) > millis();
  }

  // If sensor wasnt cut before then return false
  return false;
}

void resetSensor()
{
  sensorCutAt = 0;
}


// boolean sensorIsTrue()
// {
//   return true;
// }

///////////////////////////////////////
boolean sensorIsTrue()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(100);

  return cm <= 10;
}
///////////////////////////////////////
///////////////////////////////////////
  
long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 /2;  
}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
