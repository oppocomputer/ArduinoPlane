// Libs and Definers
#include <Arduino.h>
#include <Servo.h> //Servo's
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

//Debug Bool Delete to optimize
bool debug = true;

//Remapping Controls
const int controlRange1 = 0;
const int controlRange2 = 255;
int servoRange1 = 0;
int servoRange2 = 54;

//Servo Implementation
Servo wingLeft;
Servo wingRight;
Servo vertStabilizer;
Servo horiStabilizer;

//Pin Settings DONE
int servoPin1 = 6;
int servoPin2 = 9;
int servoPin3 = 10;
int servoPin4 = 11;

//Servo 
int pos1 = 27;  //NEEDS MANUAL ADJUSTMENT
int pos2 = 27;
int pos3 = 27;
int pos4 = 27;

//Servo Inputs
int inputPos1 = 512; //NEEDS MANUAL ADJUSTMENT
int inputPos2 = 512;
int inputPos3 = 512;
int inputPos4 = 512;


RF24 radio(10, 9);  
const byte address[6] = "10101";
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data;


void resetData() {
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}

void checkTimeData(){
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    lastReceiveTime = millis(); 
  }
  currentTime = millis();
  if (currentTime - lastReceiveTime > 1000) { 
    resetData();}
}
void applyInputData(){
  data.j1PotX = inputPos1;
  data.j1PotY = inputPos2;
  data.j2PotX = inputPos3;
  data.j2PotY = inputPos4;

}

void correctInputData(){
  pos1 = map(inputPos1, controlRange1, controlRange2, servoRange1, servoRange2);
  pos2 = map(inputPos2, controlRange1, controlRange2, servoRange1, servoRange2);
  pos3 = map(inputPos3, controlRange1, controlRange2, servoRange1, servoRange2);
  pos4 = map(inputPos4, controlRange1, controlRange2, servoRange1, servoRange2);
}

void writeFinalData(){
  wingLeft.write(pos1);
  wingRight.write(pos2);
  vertStabilizer.write(pos3);
  horiStabilizer.write(pos4);
}

void gyroDataInit(){
  if(Wire.available()){
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,14,true);

      //Bit change the readings
        AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
        AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}}

void dataPrintSerial(){
   if(Wire.available()){
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      Serial.print(" | Temperature = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | X = "); Serial.print(GyX);
      Serial.print(" | Y = "); Serial.print(GyY);
      Serial.print(" | Z = "); Serial.println(GyZ);
   } 
      Serial.print("Wing Left: ");
      Serial.print(pos1);
      Serial.println();
      Serial.print("Wing Right: ");
      Serial.print(pos2);
      Serial.println();
      Serial.print("Vertical Stabalizer: ");
      Serial.print(pos3);
      Serial.println();
      Serial.print("Horizontal Stabilizer: ");
      Serial.print(pos4);
      Serial.println();
}


void setup() {
  //USB Debug DISABLE WHEN UPLOADING
    Serial.begin(9600);

  //Receiver
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

  //Gyroscope
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

  //Servo's
    wingLeft.attach(servoPin1);
    wingRight.attach(servoPin2);
    vertStabilizer.attach(servoPin3);
    horiStabilizer.attach(servoPin4);
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening(); //  Set the module as receiver
    resetData();
}

void loop() {
    //Reset Data if no connection
    checkTimeData();

    //Apply data to Input data
    applyInputData();

    //Correct the Input data
    correctInputData();

    //Write final data to servo's
    writeFinalData();
    //Debug
     while(debug){
      gyroDataInit();
      dataPrintSerial();
      }}
        