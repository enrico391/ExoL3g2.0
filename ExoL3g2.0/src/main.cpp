#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <motorAK.h>
#include <PID.h>
#include <controller.h>
#include <svmML.h>
#include <Wire.h>

//struct can_frame canMsg;

//MCP2515 mcp2515(10);


/////////////////////////////////////////////VARIABLES//////////////////////////////////////////////
int a0_k = -2246 ;
int a1_k = -385.9;
int b1_k = -2508;
int a2_k = 3512;  
int b2_k = -1304;  
int a3_k = -128.2;
int b3_k = 1141; 
double w_k = 0.004727;

int a0_h = 3764 ;
int a1_h = -1869;
int b1_h = 1134;
int a2_h = 816.8;  
int b2_h = -500.7;  
double w_h = 0.004734;
int hip;
int knee;
unsigned long x;

int a = 0;
bool button = true;
int end = 0;

//variables of speed, position and torque
float speed;
float position;
float torque;

Controller ct;

void joystickMode(byte direction);
void checkModeBluetooth(byte mode);
void freeMode();
void gyroMode();
void PIDmode(int posHip, int posKnee);
void motorPosition(int typeMotor,int posMotor, int speedMotor);
void setMode(int posKnee, int posHip);
void writeBluetooth(String message);
String readMessageBluetooth();
void logStatus(int level);
void calibrationMode();


///////////////////////////////////////////////END///////////////////////////////////////////////////

void setup() {
  //define pin led controller 
  //define serial
  Serial.begin(9600);
  Serial3.begin(9600); // --> serial per bluetooth 
  initDriverBUS();
  //define pin bluetooth
  //define other things

}

void loop() {
  //if controllo bluetooth
  if(1==1){ //pin bluetooth attivo --> vuol dire che telefono collegato
    readMessageBluetooth();
    //condizioni
  }else{
    //mode con controller 
    switch (ct.getMode()){
      case 1:
        gyroMode();  // attraverso ML attiva e disattiva motori e curva 
        break;
      case 2:
        joystickMode(ct.getDirectionY());
        break;
      case 3:
        freeMode(); // disabilità motori per movimento libero
        break;
      case 5:
        calibrationMode(); // attraverso controller calibra i motori
        break;
      case 6:
        //pauseMode();
        break;
      default:
        break;
    }
  }
  
  ct.checkModeController();
  Serial.println(ct.getMode());
}





///////////////////////functions////////////////////////////////////////

//function with joystick and move motor
void joystickMode(byte direction){
    if(direction==0){
      //fermo
      freeMode();

    }
    else if(direction == 1){
      // avanti
      

    }else if(direction == 2){
      // indietro
    }
}

// per controllare modalità
void checkModeBluetooth(byte mode){  
    switch (mode){
        case 1:
        freeMode();
        break;
      case 2:
        gyroMode();
        break;
      case 3:
        //PIDmode();
        break;
      default:
        break;
    }
}

// mode libera disabilità motori
void freeMode(){ 
    disableMotor(1);
    disableMotor(2);
}

// mode with gyroscope
void gyroMode(){ 

}

// controllo motori con PID !!probabile non serva!!
void PIDmode(int posHip, int posKnee){  
    PidControl pidHip;
    PidControl pidKnee;

    x = 10*(a);
    a++;
    //curve
    hip =  (a0_h + a1_h*cos(x*w_h) + b1_h*sin(x*w_h) + a2_h*cos(2*x*w_h) + b2_h*sin(2*x*w_h));
    knee = a0_k + a1_k*cos(x*w_k) + b1_k*sin(x*w_k) + a2_k*cos(2*x*w_k) + b2_k*sin(2*x*w_k) + a3_k*cos(3*x*w_k) + b3_k*sin(3*x*w_k);

    //calcolo PID
    long currT = micros();
    int speedKnee = pidKnee.calculate(currT, knee, posKnee); 
    int speedHip = pidHip.calculate(currT, hip, posHip);
    motorPosition(1,speedHip,50);
    motorPosition(2,speedKnee,50);// controllare velocità e posizione

    /*
      controllo pid utilizzato in gyroMode, quando si stoppa x torna a 0 e...
    */
}

//controllare motori durante la marcia
void motorPosition(int typeMotor,int posMotor, int speedMotor){ 
    sendToMotor(typeMotor,posMotor,speedMotor,1,1,1);
}

// settare posizione iniziale
void setMode(int posKnee, int posHip){ 
    sendToMotor(1,posHip,50,1,1,1);
    sendToMotor(2,posKnee,50,1,1,1);
}

//send messages to app
void writeBluetooth(String message){
  
}

//read messages from App
String readMessageBluetooth(){
    if(Serial3.available()){
      String mode = Serial3.read();
    }
    //return byte ;
}

//print value in serial
void logStatus(int level){ 
    switch (level){
      case 1:

        break;
      case 2:

        break;
    }
      

}

// function to calibrate initial position
void calibrationMode(){
  int current_ID;
  read_fromMotor(current_ID,position,speed,torque);
  if(current_ID){
    if(ct.getDirectionX()){
      sendToMotor(current_ID,position+1,10,0,0,0); // ultimi 4 valori da testare
    }
    else if(ct.getDirectionX()==2){
      sendToMotor(current_ID,position-1,10,0,0,0); // ultimi 4 valori da testare
    }else{
      //nulla sta fermo
    }
  }else{
    if(ct.getDirectionY()){
      sendToMotor(current_ID,position+1,10,0,0,0); // ultimi 4 valori da testare
    }
    else if(ct.getDirectionY()==2){
      sendToMotor(current_ID,position-1,10,0,0,0); // ultimi 4 valori da testare
    }else{
      //nulla sta fermo
    }
  }
}
