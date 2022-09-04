#include <Arduino.h>


//library mpu

//library mcp2515
#include <SPI.h>
#include <mcp2515.h>
#include <motorAK.h>
#include <PID.h>
#include <controller.h>
#include <svmML.h>
//library wire
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

Controller ct;




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
  // put your main code here, to run repeatedly:
  //if controllo bluetooth
  if(){ //pin bluetooth attivo --> vuol dire che telefono collegato
    readMessageBluetooth();
    //condizioni
  }else{
    //mode con controller 
    switch ct.getMode(){
      case 1:
        gyroMode();
        break;
      case 2:
        joystickMode(ct.getDirection());
        break;
      case 3:
        freeMode();
        break;
      case 5:
        calibrationMode();
        break;
      case 6:
        pauseMode();
        break;
      default:
        break;
    }
  }
  
  ct.checkModeController();
  Serial.println(ct.getMode());
}


//functions

void joystickMode(byte direction){
    if(direction==0){
      //fermo
    }
    else if(direction == 1){
      // avanti
      

    }else if(direction == 2){
      // indietro
    }
}

void checkModeBluetooth(byte mode){  // per controllare modalità
    switch mode:
      case 1:
        freeMode();
        break;
      case 2:
        gyroMode();
        break;
      case 3:
        PIDmode();
        break;
      default:
        break;
}

void freeMode(){ // mode libera
    sendToMotor();
    sendToMotor();
}

void gyroMode(){ // mode with gyroscope

}

void PIDmode(int posHip, int posKnee){  // controllo motori con PID
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

void motorPosition(int typeMotor,int posMotor, int speedMotor){ //controllare motori durante la marcia
    sendToMotor(typeMotor,posMotor,speedMotor,1,1,1);
}


void setMode(int posKnee, int posHip){ // settare posizione iniziale
    sendToMotor(1,posHip,50,1,1,1);
    sendToMotor(2,posKnee,50,1,1,1);
}

void writeBluetooth(String message){
  
}

String readMessageBluetooth(){
    if(Serial3.available()){
      String mode = Serial3.read();
    }
    //return byte ;
}

void logStatus(int level){ //print value in serial
    switch (level){
      case 1:

        break;
      case 2:

        break;
    }
      

}
