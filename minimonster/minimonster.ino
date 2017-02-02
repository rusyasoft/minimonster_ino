#include "CurieIMU.h"
#include "CurieBLE.h"
#include "MadgwickAHRS.h"

BLEPeripheral blePeripheral;
BLEService MMService("83692593-7bab-42af-b186-ae389b44a01f");
BLEUnsignedCharCharacteristic reqChar("83692593-7bab-42af-b186-ae389b44a01e", BLEWrite);
BLEUnsignedLongCharacteristic dataChar("2578ad1d-105a-437f-9490-63f4cc97446a", BLENotify);

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
int flag_broken_orientation = 0;
int SKIP_SOUND = 10;
int skip_sound = 0;
int prev_heading = -1;
int SKIP_HEADING = 10;
int skip_heading = 0;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
unsigned long microsNow;

int currentCount = 0;

long currentMMData = 0;
long lastMMData = 0;

static void eventCallback(void) {
  if(CurieIMU.stepsDetected()){
    currentCount++;
  }
}

/* Handler Function */
void connect(BLECentral &central){
  init();
  CurieIMU.attachInterrupt(eventCallback);
  CurieIMU.interrupts(CURIE_IMU_STEP);
}
void disconnect(BLECentral &central){
  Serial.println("bye");
  CurieIMU.detachInterrupt();
}
void init(){
  reqChar.setValue(0);
  dataChar.setValue(0);
  currentMMData = 0;
  lastMMData = 0;
  currentCount = 0;
}

/* Blancing Function */
float convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
float convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void setup() {
  Serial.begin(9600);

  CurieIMU.begin();
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);
  CurieIMU.setStepCountEnabled(true);
  //CurieIMU.attachInterrupt(eventCallback);
  //CurieIMU.interrupts(CURIE_IMU_STEP);

  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setGyroRange(250);
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  blePeripheral.setLocalName("dumbbel");
  blePeripheral.setAdvertisedServiceUuid(MMService.uuid());
  blePeripheral.addAttribute(MMService);
  blePeripheral.addAttribute(reqChar);
  blePeripheral.addAttribute(dataChar);
  BLEPeripheralEventHandler connectHandler, disconnectHandler;

  connectHandler = connect;
  disconnectHandler = disconnect;
  blePeripheral.setEventHandler(BLEConnected, connectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, disconnectHandler);

  blePeripheral.begin();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  blePeripheral.poll();
  Serial.println("In loop!");
  delay(500);
  if(blePeripheral.central()){
    Serial.println("In Connection");
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading) {
      CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

      ax = convertRawAcceleration(aix);
      ay = convertRawAcceleration(aiy);
      az = convertRawAcceleration(aiz);
      gx = convertRawGyro(gix);
      gy = convertRawGyro(giy);
      gz = convertRawGyro(giz);

      filter.updateIMU(gx, gy, gz, ax, ay, az);

      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();
  
      if (pitch < 0 || pitch > 10){
        if (pitch < 0) {
          flag_broken_orientation = (abs((int)pitch) / 10) * 2 + ((int)abs(pitch) % 10);
        }
        else {
          flag_broken_orientation = ( (int)pitch / 10) *2 + ((int)abs(pitch) % 10);
        }
      } else flag_broken_orientation = 0;

      if (flag_broken_orientation!=0 && skip_sound==0){
        Serial.println("Orientation incorrect");
        if (pitch > 0) tone(11, 500*(1+(int)(pitch-10) / 10), 100);
        else tone(11, 3000*(1+(int)abs(pitch) / 10), 100);   
        skip_sound = SKIP_SOUND;
      }
      if (skip_sound>0) skip_sound-=flag_broken_orientation;
      if (skip_sound<0) skip_sound = 0;
      microsPrevious = microsPrevious + microsPerReading;
    }
    currentMMData = (flag_broken_orientation<<8) | (0xFF & currentCount);
    if(currentMMData != lastMMData){
      dataChar.setValue((flag_broken_orientation<<8) | (0xFF & currentCount) );
      lastMMData = currentMMData;
    }
  }
}
