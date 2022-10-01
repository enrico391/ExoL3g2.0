
#include <mcp_can.h>
#include <SPI.h>

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



MCP_CAN CAN(10);

void checkCAN(){
    while(CAN_OK != CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ))
    {
        Serial.println("CAN BUS FAIL");
        Serial.println("TRY AGAIN...");
        delay(100);
    }
    Serial.println("CAN BUS init ok");

    CAN.setMode(MCP_NORMAL);
}




float uint_to_float(int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
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

void enterMotorMode(byte id){
    byte buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    buf[5] = 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFC;

    byte sndStat = CAN.sendMsgBuf(id, 0, 8, buf);

    if(sndStat == CAN_OK) Serial.println("Message Sent Successfully!");
    else Serial.println("Error Sending Message...");
  
}


void exitMotorMode(byte id ){
    byte buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    buf[5] = 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFD;

    byte sndStat = CAN.sendMsgBuf(id, 0, 8, buf);
    
    if(sndStat == CAN_OK) Serial.println("Message Sent Successfully!");
    else Serial.println("Error Sending Message...");
}


void setZeroMotor(byte id){
    byte buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = 0xFF;
    buf[5] = 0xFF;
    buf[6] = 0xFF;
    buf[7] = 0xFE;

    byte sndStat = CAN.sendMsgBuf(id, 0, 8, buf);
    
    if(sndStat == CAN_OK) Serial.println("Message Sent Successfully!");
    else Serial.println("Error Sending Message...");
}


void sendToMotor(int id, float pos, float speed, float kp, float kd, float torq){
    byte buf[8];

    unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
    unsigned int con_vel = float_to_uint(constrain(speed, V_MIN, V_MAX), V_MIN, V_MAX, 12);
    unsigned int con_kp = float_to_uint(constrain(kp, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
    unsigned int con_kd = float_to_uint(constrain(kd, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
    unsigned int con_torq = float_to_uint(constrain(torq, T_MIN, T_MAX), T_MIN, T_MAX, 12);

    buf[0] = con_pos >> 8;
    buf[1] = con_pos & 0xFF;
    buf[2] = con_vel >> 4;
    buf[3] = ((con_vel & 0xF) << 4) | (con_kp>>8);
    buf[4] = con_kp & 0xFF;
    buf[5] = con_kd >> 4;
    buf[6] = ((con_kd & 0xF)<<4) | (con_torq >> 8);
    buf[7] = con_torq & 0xFF;

    byte sndStat = CAN.sendMsgBuf(id, 0, 8, buf);
    
    if(sndStat == CAN_OK) Serial.print(" Message Sent Successfully!");
    else Serial.print(" Error Sending Message...");
}



void read_fromMotor(long unsigned int &id,float &pos, float &speed, float &torque){
    byte len = 0;
    byte buf[8] = {};
    //unsigned char len = 0;
    //unsigned char buf[8];
    
    CAN.readMsgBuf(&id,&len,buf);

    int Upos = (buf[1] << 8) | buf[2]; //Motor position data
    int Uspeed = (buf[3] << 4) | (buf[4] >> 4); // Motor speed data
    int Utorque = ((buf[4] & 0xF) <<8) | buf[5]; // Motor torque data

    pos = uint_to_float(Upos, P_MIN, P_MAX, 16);
    speed = uint_to_float(Uspeed, V_MIN, V_MAX, 12);
    torque = uint_to_float(Utorque, -T_MAX, T_MAX, 12);
}