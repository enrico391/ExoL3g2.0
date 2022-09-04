
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"


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


//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;


//This code was autogenerated using micro-learn.
//Use the dummy variable array 'data' to interact with the ML inference.
/* dati per ginocchio chiuso o aperto
double w[] = {-0.025649630081589908, 0.1090756220091944, 1.6423935239866403, 0.2626203785638569};
double u[] = {15234.850980392157, -10.008823529411766, 8866.462745098039, 48.311764705882354};
double p[] = {0.0032605602651013064, 0.002830807503403701, 0.0002137646204489013, 0.0011259078088872872};

double c = 0.6191725127672563;
*/

double w[] = {-1.5150847909302836, 0.058820816356530164, -1.1350316251028203, -0.2881538602105478};
double u[] = {-535.2365315546434, -1.0615700359158542, 12.096459722934839, 0.9907644946126218};
double p[] = {0.0007629927776745341, 0.005105650876470956, 0.0008388574130320697, 0.007478236073259843};

double c = 0.525712033288551;



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
byte numReads = 1;


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

}




void loop() {
    float data[4]; //This is your feature vector. Retrive your data into this array.
    //Data Section: To Be Coded Manually
    //to smooth curves
    mpu1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    data[0] = ay;
    data[1] = gy;
    data[2] = ay2;
    data[3] = gy2;
    
    
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        //Serial.print("MPU 1 :");
        //Serial.print(ay);// o ay 
        //Serial.print(" ");Serial.println(ay2); // !!
    #endif
    

    //ML Inference Section

    double temp = 0.0;
    for(int i=0; i<4; i++)
    {
        temp += (data[i]-u[i]) * p[i] * w[i];
    }
    Serial.println(temp);
    if(temp >= c)
    {
        //Do something for class label 1.
        Serial.println("Fermo");
    }
    else
    {
        //Do something for class label 0.
        Serial.println("In camminata"); 
    }

    
}
