#include <Arduino.h>


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SPI.h>
#include <SD.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

File dataSet;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/* orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

*/

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
int senseSumAccY_hip = 0;
int senseSumAccY_knee = 0;
int senseSumGyroY_knee = 0;
int senseSumGyroY_hip = 0;
byte numReads = 30;


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu1.initialize();
    mpu2.initialize();

    
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu1.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println(mpu2.testConnection() ? "MPU6050 2 connection successful" : "MPU6050 2 connection failed");


      // Open serial communications and wait for port to open:
    
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  
  
    Serial.print("Initializing SD card...");
  
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    //myFile = SD.open("valori.txt", FILE_WRITE);
    
    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets MPU 1...");
    // -76  -2359 1688  0 0 0
    Serial.print(mpu1.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(mpu1.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(mpu1.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(mpu1.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu1.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu1.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    /*
    mpu1.setXAccelOffset(-3238);
    mpu1.setYAccelOffset(1638);
    mpu1.setZAccelOffset(984);
    mpu1.setXGyroOffset(66);
    mpu1.setYGyroOffset(18);
    mpu1.setZGyroOffset(32);
    */
    mpu1.setXAccelOffset(-1385);
    mpu1.setYAccelOffset(1553);
    mpu1.setZAccelOffset(2614);
    mpu1.setXGyroOffset(65);
    mpu1.setYGyroOffset(18);
    mpu1.setZGyroOffset(32);
    Serial.print(mpu1.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(mpu1.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(mpu1.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(mpu1.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu1.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu1.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");

    Serial.println("Updating internal sensor offsets MPU 2...");
    // -76  -2359 1688  0 0 0
    Serial.print(mpu2.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(mpu2.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(mpu2.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(mpu2.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu2.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu2.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    /*
    mpu2.setXAccelOffset(743 );
    mpu2.setYAccelOffset(1056 );
    mpu2.setZAccelOffset(993);
    mpu2.setXGyroOffset(15);
    mpu2.setYGyroOffset(112);
    mpu2.setZGyroOffset(62);
    */
    mpu2.setXAccelOffset(2300 );
    mpu2.setYAccelOffset(1065 );
    mpu2.setZAccelOffset(3115);
    mpu2.setXGyroOffset(15);
    mpu2.setYGyroOffset(112);
    mpu2.setZGyroOffset(57);
    Serial.print(mpu2.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(mpu2.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(mpu2.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(mpu2.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu2.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu2.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    delay(2000);



    dataSet = SD.open("dataSet.txt", FILE_WRITE);
    if (dataSet) {
      dataSet.println(""time","timeReal","AccYHip","GyroYHip","AccYKnee","GyroYKnee","Occupancy""); // ay2 gy2  --> ginocchio,   ay gy --> anca
      }
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening dataSet.txt");
    }
    dataSet.close();
}

int index = 0;
byte occupancy = 0;
void loop() {
    // read raw accel/gyro measurements from device
    occupancy = digitalRead(2);
    mpu1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    //to smooth curves
     for(byte k = 0;k<numReads; k++){
      senseSumAccY_hip +=ay;
      delay(1);
      }
    ay = senseSumAccY_hip / numReads;

    for(byte k = 0;k<numReads; k++){
      senseSumGyroY_hip +=gy;
      delay(1);
      }
    gy = senseSumGyroY_hip / numReads;

    for(byte k = 0;k<numReads; k++){
      senseSumAccY_knee +=ay2;
      delay(1);
      }
    ay2 = senseSumAccY_knee / numReads;

    for(byte k = 0;k<numReads; k++){
      senseSumGyroY_knee +=gy2;
      delay(1);
      }
    gy2 = senseSumGyroY_knee / numReads;

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("MPU 1 :");
        Serial.print(" ");Serial.print(ay);// o ay 
        Serial.print(" ");Serial.println((ay2)); // !!
    #endif

    dataSet = SD.open("dataSet.txt", FILE_WRITE);
    if (dataSet) {
      dataSet.print(index +"," millis() + "," + ay + "," + gy + "," + ay2 + "," + gy2 + "," + occupancy );
      dataSet.println("");
      }
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening dataSet.txt");
    }
    dataSet.close();
}
