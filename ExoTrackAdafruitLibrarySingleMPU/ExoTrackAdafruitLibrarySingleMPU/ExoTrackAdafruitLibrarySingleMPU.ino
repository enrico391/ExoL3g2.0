

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;


void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(9600);

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }

    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
    Serial.print("Accelerometer range set to: ");
  
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
  
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
    Serial.print("Gyro range set to: ");
  
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }
  
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
    Serial.print("Filter bandwidth set to: ");
  
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
  
    Serial.println("");
    delay(100);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Initializing SD card...");
  
    
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    
    // if the file opened okay, write to it:
    Serial.println("\"index\",\"timeReal\",\"Ax\",\"Ay\",\"Az\",\"Gx\",\"Gy\",\"Gz\",\"Occupancy\"");
}




int index = 0;
byte occupancy = 0;

void loop() {
    occupancy = digitalRead(2);
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print(index);
    Serial.print(" , ");
    Serial.print(a.acceleration.x);
    Serial.print(" , ");
    Serial.print(a.acceleration.y);
    Serial.print(" , ");
    Serial.print(a.acceleration.z);
    Serial.print(" , ");
    Serial.print(g.gyro.x);
    Serial.print(" , ");
    Serial.print(g.gyro.y);
    Serial.print(" , ");
    Serial.print(g.gyro.z);
    Serial.print(" , ");
    Serial.println(occupancy);
    
    index++;
}
