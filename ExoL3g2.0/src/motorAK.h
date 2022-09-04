#include <Arduino.h>

//library mcp2515
#include <SPI.h>
#include <mcp2515.h>

 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -20.94f
 #define V_MAX 20.94f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -24.8f
 #define T_MAX 24.8f

struct can_frame canMsg;

MCP2515 mcp2515(10);
/*
void setup() {

  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  enterMotorMode(1);
  enterMotorMode(2);
  delay(5000);

}

void loop() {


  sendToMotor(1, 0, -6.28, 0, 5, 0);
  sendToMotor(2, 0, 6.28, 0, 5, 0);
  delay(3000);
  sendToMotor(1, 0, 6.28, 0, 3, 0);
  sendToMotor(2, 0, 6.28, 0, 3, 0);
  delay(1200);

}
*/

void enterMotorMode(int mot_id){
  struct can_frame cf;
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = 0xFF;
  cf.data[1] = 0xFF;
  cf.data[2] = 0xFF;
  cf.data[3] = 0xFF;
  cf.data[4] = 0xFF;
  cf.data[5] = 0xFF;
  cf.data[6] = 0xFF;
  cf.data[7] = 0xFC;
  mcp2515.sendMessage(&cf);
}

void disableMotor(int mot_id){
  //motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD]
  struct can_frame cf;
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = 0xFF;
  cf.data[1] = 0xFF;
  cf.data[2] = 0xFF;
  cf.data[3] = 0xFF;
  cf.data[4] = 0xFF;
  cf.data[5] = 0xFF;
  cf.data[6] = 0xFF;
  cf.data[7] = 0xFD;
}

void setZeroMotor(int mot_id){
  //(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
  struct can_frame cf;
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = 0xFF;
  cf.data[1] = 0xFF;
  cf.data[2] = 0xFF;
  cf.data[3] = 0xFF;
  cf.data[4] = 0xFF;
  cf.data[5] = 0xFF;
  cf.data[6] = 0xFF;
  cf.data[7] = 0xFE;
}



void initDriverBUS(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  //enterMotorMode(1);
  //enterMotorMode(2);
  delay(5000);
}


int float_to_uint(float x, float x_min, float x_max, int bits){
    // Converts a float to an unsigned int, given range and number of bits 
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if(bits == 12){
      pgg = (unsigned int) ((x-offset)*4095.0/span);
    }else if(bits == 16){
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
      /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void sendToMotor(int mot_id, float pos, float vel, float kp, float kd, float torq){
  struct can_frame cf;

  unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
  unsigned int con_vel = float_to_uint(constrain(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
  unsigned int con_kp = float_to_uint(constrain(kp, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
  unsigned int con_kd = float_to_uint(constrain(kd, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
  unsigned int con_torq = float_to_uint(constrain(torq, T_MIN, T_MAX), T_MIN, T_MAX, 12);
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = con_pos>>8;
  cf.data[1] = con_pos & 0xFF;
  cf.data[2] = con_vel >> 4;
  cf.data[3] = ((con_vel&0xF)<<4) | (con_kp>>8);
  cf.data[4] = con_kp&0xFF;
  cf.data[5] = con_kd>>4;
  cf.data[6] = ((con_kd&0xF)<<4) | (con_torq>>8);
  cf.data[7] = con_torq&0xFF;
  mcp2515.sendMessage(&cf);
}

void read_fromMotor(int& idMotor,float& positionCurrent, float& speedCurrent, float& torqueCurrent){
  struct can_frame msg;
  if(mcp2515.readMessage(&msg) == MCP2515::ERROR_OK){
    /// unpack ints from can buffer ///
    idMotor = msg.data[0]; //id motor
    int p_int = (msg.data[1]<<8)|msg.data[2]; //Motor position data
    int v_int = (msg.data[3]<<4)|(msg.data[4]>>4); // Motor speed data
    int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5]; // Motor torque data
    /// convert ints to floats ///
    positionCurrent = uint_to_float(p_int, P_MIN, P_MAX, 16);
    speedCurrent = uint_to_float(v_int, V_MIN, V_MAX, 12);
    torqueCurrent = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  }
}


