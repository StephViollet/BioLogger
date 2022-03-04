/* Dépendances */
#include <Arduino_LSM9DS1.h> //IMU
#include <MadgwickAHRS.h> // Euler angle estimation
#include <SPI.h> // SPI communication 
#include <SD.h>  // SD communication 
#include <ArduinoBLE.h> // Bluetooth BLE communication 
#include "RTClib.h" // RTC (real time clock circuit) communication

/** Variables to be saved on the SD card */
float ax,ay,az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, heading;


/** S=STOP G=GO */
char inchar;
bool trig_imu = false;
bool trig_BLE = false;
bool flag = true;
byte var_state;
int val = 0;
int compteur  = 0;

//const int ledPin = LED_BUILTIN; // pin to use for the LED

/** Pin CS SD card */
const byte SDCARD_CS_PIN = 10; // A remplacer suivant votre shield SD

/** Output file Name */
//const char* OUTPUT_FILENAME = "data.csv";

/** Time between two samples(ms) */
//unsigned long microsPerReading, microsPrevious;
//unsigned long microsNow;
unsigned long millisPerReading, millisPrevious;
unsigned long millisNow;
float Angles[3];
float Angles_null[3];

/** Output file  */
File file;

/** Config RTC */
RTC_PCF8523 rtc;

/** Config type of filter for Euler angle estimation */
// Madgwick
Madgwick filter;

//UUID generator : https://www.uuidgenerator.net
BLEService Logger("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service
//Buffer size  = 4 x 4octets (3 float)  = 12 octets
BLECharacteristic LoggerRead("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 12);
//BLEService Logger1("c539fd66-c5f4-11ea-87d0-0242ac130003"); // create service


// create switch characteristic and allow remote device to read and write LOGGER1
BLEUnsignedCharCharacteristic LoggerWrite("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLENotify);
//BLEUnsignedCharCharacteristic LoggerRead("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// create switch characteristic and allow remote device to read and write LOGGER2
//BLEUnsignedCharCharacteristic LoggerWrite("c539ffe6-c5f4-11ea-87d0-0242ac130003", BLEWrite | BLENotify);
//BLEUnsignedCharCharacteristic LoggerRead("c53a00e0-c5f4-11ea-87d0-0242ac130003", BLERead | BLENotify);

// RTC circuit : PCF8523
//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
 DateTime now = rtc.now();

 // return date using FAT_DATE macro to format fields
 *date = FAT_DATE(now.year(), now.month(), now.day());

 // return time using FAT_TIME macro to format fields
 *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
//------------------------------------------------------------------------------

void setup() {
  
/* Initialisation serial port (debug) */
  Serial.begin(9600);

  /* Initialisation SPI port */
  //pinMode(ledPin, OUTPUT);

  /* Initialisation SD card */
  Serial.println(F("Initialisation de la carte SD ... "));
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println(F("Erreur : Impossible d'initialiser la carte SD"));
    Serial.println(F("Verifiez la carte SD et appuyez sur le bouton RESET"));
    for (;;); // Attend appui sur bouton RESET
     }
   delay(1000);
     
if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Used to write a file with correct time and date
  // set date time callback function
 SdFile::dateTimeCallback(dateTime); 

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    //If uncomment used to set the time from the PC
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }

  delay(1000);

   if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //Initialise Angles null
  Angles_null[0] = 0;
  Angles_null[1] = 0;
  Angles_null[2] = 0;

/*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     ****************
********************         Copy/Replace the lines below by the code output of the program              ****************/
   IMU.setAccelFS(0);
   IMU.setAccelODR(3);
   IMU.setAccelOffset(-0.014135, -0.021674, 0.001026);
   IMU.setAccelSlope (0.996814, 0.999071, 1.008192);
/***********************************************************************************************************************************
*******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
*******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******
************************************************************************************************************************************/
IMU.accelUnit=  GRAVITY;    // or  METERPERSECOND2    

/*******   The gyroscope needs to be calibrated. Offset controls drift and Slope scales the measured rotation angle  *********
*****************   Copy/Replace the lines below by the output of the DIY_Calibration_Gyroscope sketch   ********************/
   IMU.setGyroFS(1);
   IMU.setGyroODR(3);
   IMU.setGyroOffset (-0.107498, 0.559113, 0.656723);
   IMU.setGyroSlope (1.0000, 1.0000, 1.0000);
/*****************************************************************************************************************************     
*********  FS  Full Scale       setting 0: ±245°/s | 1: ±500°/s | 2: ±1000°/s | 3: ±2000°/s       ****************************
*********  ODR Output Data Rate setting 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (not working 6:952Hz)   *******
*****************************************************************************************************************************/ 
IMU.gyroUnit= DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  

/*****************   For a proper functioning of the magnetometer it needs to be calibrated            ********************
*****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/
   IMU.setMagnetFS(0);
   IMU.setMagnetODR(8);
   IMU.setMagnetOffset(-0.961304, 17.479858, 52.926636);
   IMU.setMagnetSlope (1.726894, 1.652259, 1.380619);
/******************************************************************************************************************************     
****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****     
****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
*******************************************************************************************************************************/    
 IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA

  BLE.begin();

// set the local name peripheral advertises
  BLE.setLocalName("LOG1");
  //BLE.setLocalName("LOG2");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(Logger);
  //BLE.setAdvertisedService(IMUNanoRead);

  // add the characteristic to the service
  Logger.addCharacteristic(LoggerWrite);
  Logger.addCharacteristic(LoggerRead);
  
  // add service
  BLE.addService(Logger);
  //BLE.addService(LoggerRead);
  
  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  LoggerWrite.setEventHandler(BLEWritten, LoggerWriteWritten);
  LoggerRead.setEventHandler(BLERead, ReadLogger);
  
  // set an initial value for the characteristic
  LoggerWrite.setValue(0);
  LoggerRead.setValue(0);

  // start advertising
  BLE.advertise();
  Serial.println(("BLE LOG1 Peripheral"));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();
  
/** Timestamp = 10ms */
    millisPerReading = 10;
    millisPrevious = millis();
    filter.begin(20);

  
}

void loop() {

   // poll for BLE events
  BLE.poll();
  
  //microsNow = micros();
  millisNow = millis();

//  if (Serial.available() > 0) 
//        inchar = Serial.read();

//  if ((inchar == 'G') && (flag)) {
//    flag = false;
//    create_file();
//    }
//    
//  if ((inchar == 'S') && (!flag)) {
//    trig = false;
//    flag = true;
//    file.close();
//    }

  if ((trig_BLE) && (flag)) {
    flag = false;
    create_file();
    //Lance l'IMU pour l'enregistrement après avoir crée le fichier
    trig_imu = true;
    }
    
  if ((!trig_BLE) && (!flag)) {
    // Stop IMU
    trig_imu = false;
    flag = true;
    //file.flush();
    file.close();
    }

  //trig_imu = true;
  if ((millisNow - millisPrevious >= millisPerReading) && (trig_imu)) {
    
    millisPrevious = millisNow;
    measure_IMU();
    writeSD();
    
    compteur++;
    if (compteur == 1000) {
    compteur = 0;
    file.flush();
    }

// Uncomment to monitor via the serial port
//   Serial.print(Angles[0]);
//   Serial.print(",");
//   Serial.print(Angles[1]);
//   Serial.print(",");
//   Serial.println(Angles[2]);
    
  }
}

/** Data acquisition function */
void writeSD() {
/* Enregistre les données sur la carte SD */
  file.print(millisPrevious);
  file.print(F("; "));
  file.print(ax);
  file.print(F("; "));
  file.print(ay);
  file.print(F("; "));
  file.print(az);
  file.print(F("; "));
  file.print(gx);
  file.print(F("; "));
  file.print(gy);
  file.print(F("; "));
  file.print(gz);
  file.print(F("; "));
  file.print(mx);
  file.print(F("; "));
  file.print(my);
  file.print(F("; "));
  file.print(mz);
  file.print(F("; "));
  file.print(Angles[0]); //Roll
  file.print(F("; "));
  file.print(Angles[1]); //Pitch
  file.print(F("; "));
  file.print(Angles[2]); //Yaw
  file.println(F("; "));

  /*  Flush : takes too much time */
  //file.flush();
  
  }

void create_file() {

  DateTime now = rtc.now();

  String second = String(now.second(), DEC);
  String minute = String(now.minute(), DEC);
  String hour = String(now.hour(), DEC);

  String filename = hour + "-" + minute + "-" + second + ".CSV";
  
/* Open the output file to be written */
  Serial.println(F("Ouverture du fichier de sortie ... "));
  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Erreur : Impossible d'ouvrir le fichier de sortie"));
    Serial.println(F("Verifiez la carte SD et appuyez sur le bouton RESET"));
    for (;;); // Attend appui sur bouton RESET
  }

  /* If emply file add CSV header */
  if (file.size() == 0) {
    Serial.println(F("Ecriture de l'entete CSV ..."));
    file.println(F("Time; Acc X; Acc Y; Acc Z; Gx; Gy; Gz; Mx; My; Mz; Roll; Pitch; Head"));
    file.flush();
   
 }
 }

void measure_IMU() {

  if (IMU.accelAvailable()) {                 
  
    IMU.readAccel(ax, ay, az);
    IMU.readGyro(gx, gy, gz);
    IMU.readMagnet(mx, my, mz); 
    }

  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  Angles[0] = filter.getRoll();
  Angles[1] = filter.getPitch();
  Angles[2] = filter.getYaw();  

 }  
 
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  //digitalWrite(LED_BUILTIN, HIGH);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  //digitalWrite(LED_BUILTIN, LOW);
}

void LoggerWriteWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update TRIG
  //Serial.println("Characteristic event, written: ");

  LoggerWrite.readValue(var_state);
  //Serial.println(var_state);
  
// 65 dec : 41 en hex : letter A ASCII
  if (var_state == 65) {
    //Serial.println("TRIG on");
    trig_BLE = true;
    //digitalWrite(ledPin, HIGH);
  } 

//  90 dec : 5A en hex : Letter Z ASCII
  if (var_state == 90) {
    //Serial.println("TRIG off");
    trig_BLE = false;
    //digitalWrite(ledPin, LOW);
    Angles[0] = 0;
    Angles[1] = 0;
    Angles[2] = 0; 
    LoggerRead.setValue((byte *) &Angles, 12);
  }
   }
   
void ReadLogger(BLEDevice central, BLECharacteristic characteristic) {
      
      //Serial.println("Characteristic event, read ");
      if (trig_BLE)
        LoggerRead.setValue((byte *) &Angles, 12);
        else
          LoggerRead.setValue((byte *) &Angles_null, 12);
            
  }


  
