#include <Utils.h>
#include <ArduinoBLE.h>
#include <LSM6DS3.h>  // https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
#include <Wire.h>
#include <SD.h>

/************** STRUCTURE *****************/
const byte addressIMU = 0x6A;
LSM6DS3 lsm6ds33(I2C_MODE, addressIMU);
File dataFile;
String dataFileName;

float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float humidity;
float accelX, accelY, accelZ,                               // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                                      // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ,                       // units dps
  gyroRoll, gyroPitch, gyroYaw,                             // units degrees (expect major drift)
  gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
  accRoll, accPitch, accYaw,                                // units degrees (roll and pitch noisy, yaw not possible)
  complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)
int roll, pitch, yaw;

typedef struct {
} DataIMU, *PtrDataIMU;

DataIMU dataIMU;

enum { IDLE,
       SAVING,
        READY };
unsigned char state = IDLE;

#define DIRECTORY "SWEAT/WORKOUTS"
#define SD_PIN 3


long lastTime;
long lastInterval;

const int LedRGBOnValue = LOW;
const int LedRGBOffValue = HIGH;

/************ BLE SERVICES & CHARACTERISTICS ****************/

// Service - Current Time Service (0x1805)
const char* serviceGATT = "1805";

//const char* nameCentralBLE = "XIAO SWEAT RIGHT WRIST";
//String fileName = "/SRW";

//const char* nameCentralBLE = "XIAO SWEAT LEFT WRIST";
//String fileName = "/SLW";

//const char* nameCentralBLE = "XIAO SWEAT LEFT SHOULDER";
//String fileName = "/SLS";

const char* nameCentralBLE = "XIAO SWEAT LEFT ELBOW";
String fileName = "/SLE";


// Time service
BLEService timeService(serviceGATT);

// Swimu/Sports characteristics
const char* characteristicCurrentTimeServiceGATT = "2A2B";
const char* characteristicCurrentTimeServiceValue = "Date Time";
const int characteristicCurrentTimeServiceValueSize = 10;


BLEStringCharacteristic characteristicCurrentTimeService(characteristicCurrentTimeServiceGATT, BLEWrite, 20);
BLEDescriptor descriptorCurrentTimeService(characteristicCurrentTimeServiceGATT, characteristicCurrentTimeServiceValue);




/********************  IMU SETUP FUNCTIONS *********************/


bool readIMU() {
  accelX = lsm6ds33.readFloatAccelX();
  accelY = lsm6ds33.readFloatAccelY();
  accelZ = lsm6ds33.readFloatAccelZ();

  gyroX = lsm6ds33.readFloatGyroX();
  gyroY = lsm6ds33.readFloatGyroY();
  gyroZ = lsm6ds33.readFloatGyroZ();

  return true;
}

void calibrateIMU(int delayMillis, int calibrationMillis) {
  int calibrationCount = 0;

  delay(delayMillis);  // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    DebugMessagePrintf("Failed to calibrate!\n");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}

void doImuCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = (float)1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
if ( complementaryYaw<0.0f ) {
    complementaryYaw = complementaryYaw + 360.0f;
  }
  else {
    if ( complementaryYaw>360.0f ) {
      complementaryYaw = complementaryYaw - 360.0f;
    }
  }



 roll  = map(complementaryRoll, -180, 180, 0, 360);
  pitch = map(complementaryPitch, -180, 180, 0, 360);

  //pitch   = (int)complementaryPitch;
  //roll   = (int)complementaryRoll;
  yaw   = (int)complementaryYaw;
}


void initLED(const int ledPin) {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LedRGBOffValue);
}


void myBlink(const int ledPin, const int blinkTime) {
  digitalWrite(ledPin, LedRGBOnValue);
  delay(blinkTime);
  digitalWrite(ledPin, LedRGBOffValue);
  delay(blinkTime);
}



void onCurrentTimeServiceReceived(BLEDevice central, BLECharacteristic characteristic) {
  DebugMessagePrintf("\nReceived\n");

  if (characteristic == characteristicCurrentTimeService) {

    switch(state){

    case(SAVING):

      dataFile.close();
      Serial.println("IDLE");
      state = IDLE;
      break;

    case(READY):
      delay(2000);
      state = SAVING;
      break;

    case(IDLE):
      Serial.println("Received Value");
      uint8_t data[8];
      int bufferData = characteristic.readValue(data, 8);

      char mainDir[9];
      char secDir[7];

      sprintf(mainDir, "/%04u%02u%02u", (data[1] << 8) | data[0], data[2], data[3]);
      sprintf(secDir, "/%02u%02u%02u", data[4], data[5], data[6]);
      initSaving(mainDir, secDir, fileName);
      break;


    }
  }
}


void setupSD(){

    DebugMessagePrintf("Initializing SD card...\n");
    if(!SD.begin(SD_PIN)) {
      DebugMessagePrintf("SD card initialization failed!\n\n");
    }

    DebugMessagePrintf("Card ready, checking for necessary directories...\n");
    if(!SD.exists(DIRECTORY)) {
      if(!SD.mkdir(DIRECTORY)) {
        DebugMessagePrintf("Create Directory failed\n");
      }
      File myFile = SD.open(String(DIRECTORY) + "/tosave.txt", FILE_WRITE);
      myFile.print("0\n0");
      myFile.close();
    }

    //if(SD.rmdir(String(DATA_DIR) + "/20230419")) DebugMessagePrintf("rem\n");

    Serial.println("All ready!\n");
}

void initSaving(char* dir1, char* dir2, String file) {
  if (!SD.exists(String(DIRECTORY) + String(dir1))) {

    SD.mkdir(String(DIRECTORY) + String(dir1) + String(dir2));
    Serial.println("Created Directory");
  } else {
    Serial.println("Added Directory");
    SD.mkdir(String(DIRECTORY) + String(dir1) + String(dir2));
  }

Serial.println(String(dir1));
Serial.println(String(dir2));
Serial.println(String(file));
  //Open file to save data
  Serial.println("Opening file");
  dataFile = SD.open(String(DIRECTORY) + String(dir1) + String(dir2) + String(file) + ".txt", FILE_WRITE);
  dataFileName = dataFile.name();
  Serial.println(dataFile);
  Serial.println(dataFileName);
  if (dataFile) {
    Serial.println("Ready");
    state = READY;
  } else {

    // Error file init code
    state = IDLE;
  }
}



/************************* RUN *****************************/
void setup() {
  Serial.begin(9600);
  DebugDelay(2000);

  DebugSerialBeginNoBlock(115200);

  DebugMessagePrintf("Serial port is ready.\n");

  DebugMessagePrintf("Configuring Builtin LED...\n");
  initLED(LED_BUILTIN);

  DebugMessagePrintf("Configuring RGB LED...\n");
  initLED(LEDR);
  initLED(LEDG);
  initLED(LEDB);

  if (lsm6ds33.begin() != 0) {
    DebugMessagePrintf("Starting device LSM6DS3 failed!\n");

    // Turn on the Red LED on to indicate that there was an error on setup
    while (1) {
      myBlink(LEDR, 500);
    }
  }

  DebugMessagePrintf("Device LSM6DS3 is ready. Going to calibrate IMU...\n");

  

  if (!BLE.begin()) {
    DebugMessagePrintf("Starting Bluetooth® Low Energy module failed!\n");

    // Turn on the Red LED on to indicate that there was an error on setup
    while (1) {
      myBlink(LEDR, 1000);
    }
  }

  if (!SD.begin(SD_PIN)) {
    Serial.println("SD card initialization failed!\n\n");
  }


  DebugMessagePrintf("Bluetooth® Low Energy is ready.\n");
  DebugMessagePrintf("Setting BLE Central name to: %s\n", nameCentralBLE);
  BLE.setLocalName(nameCentralBLE);
  DebugMessagePrintf("Setting the advertise service...\n");
  BLE.setAdvertisedService(timeService);
  DebugMessagePrintf("Adding characteristics to service...\n");
  timeService.addCharacteristic(characteristicCurrentTimeService);
  DebugMessagePrintf("Adding descriptors to characteristic...\n");
  characteristicCurrentTimeService.addDescriptor(descriptorCurrentTimeService);
  DebugMessagePrintf("Preparing onWrite function...\n");
  characteristicCurrentTimeService.setEventHandler(BLEWritten, onCurrentTimeServiceReceived);
  DebugMessagePrintf("Adding service to BLE...\n");
  BLE.addService(timeService);
  DebugMessagePrintf("Going to advertise...\n");
  BLE.advertise();
  DebugMessagePrintf("Device MAC: %d\n", BLE.address().c_str());
  DebugMessagePrintf("Setup is ready.\n");
  DebugMessagePrintf("Waiting for connections in five seconds...\n\n");
  digitalWrite(LEDG, LedRGBOnValue);
  calibrateIMU(2500, 2500);
}


void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  doImuCalculations();

  switch (state) {
    case IDLE:
      if (central) {
        DebugMessagePrintf("Connected to peripheral device MAC: %s\n", central.address().c_str());
        if ( central.connected() ) BLE.poll();
        // Turn on the Blue LED on to indicate the connection
        digitalWrite(LEDB, LedRGBOnValue);
      }

      if (Serial.available()) {
        int input = Serial.parseInt();
        if (input == 1) {
          Serial.println("Enter");
          dataFile = SD.open("/SWEAT/WORKOUTS/20230824/150611/SRW.txt");
          if (dataFile) {
            while (dataFile.available()) {
              char character = dataFile.read();  // Read each character
              Serial.print(character);
            }
            Serial.println("Fim");
            dataFile.close();
          }
        }
      }
      break;

    case READY:
      break;

    case SAVING:

      if ( central ) {
        if ( central.connected() ) BLE.poll();
      }


      if (readIMU()) {
        long currentTime = micros();
        lastInterval = currentTime - lastTime;
        lastTime = currentTime;
        
      }

      if (dataFile) {

        myPrintf(dataFile, "%d;%d;%d;%d\n", millis(), roll, pitch, yaw);
        myPrintf("%d;%d;%d;%d\n", millis(), roll, pitch, yaw);

        //dataFile.print(String(roll));
        //Serial.print(String(roll));
        //Serial.print(";");
        //dataFile.print(';');
        //dataFile.print(String(pitch));
        //Serial.print(String(pitch));
        //Serial.print(";");
        //dataFile.print(';');
        //dataFile.println(String(yaw));
        //Serial.println(String(yaw));

      } else {

        Serial.println("Error opening data.txt");
      }
      break;
  }
}