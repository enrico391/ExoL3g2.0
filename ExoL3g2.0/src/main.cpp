#define PIN_BLUETOOTH 33
#include <iostream>
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <motorAK.h>
#include <PID.cpp>
#include <controller.cpp>
#include <svmML.cpp>
#include <Wire.h>
#include <curves.h>

////////////////MPU6050////////////////////
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

//oggetto per calcolo Machine Learning
Svm ml_calculator;


/////////////////////////////////////////////VARIABLES//////////////////////////////////////////////

//da rivedere
int hip;
int knee;

// risultato retta nelle curve camminata per valore x
unsigned int tempo = 1;
// x della retta per curve camminata
float xT = 0;
// fattore scala per iniziare camminata con curve piccole
float scala = 0;
//fattore scala in orizzontale
float scaleHori;


//variables of speed, position and torque
float speed = 0;
float speedKnee;
float position;
float positionKnee;
float torque;
float torqueKnee;


float id;

//oggetto controller per ricevere info su controller mode
Controller ct;

//variables for App control
byte modeBluetooth;
int speedBluetooth;





/*
mode 1 --> freeMode
mode 2 --> gyroMode
mode 3 --> PIDMode (forse no)
mode 5 --> calibrationMode
*/


////////////////////////////FUNCTIONS////////////////////////////////////////

/* declaration of functions
void joystickMode(byte direction);

void checkModeBluetooth(byte mode);
//mode libera con motori disable e libertà di movimento
void freeMode();
//mode con giroscopio e algoritmo di machine learning
void gyroMode();
//mode PID forse non serve
void PIDmode(int posHip, int posKnee);
//funzione per settare posizione motori |||forse non necessaria
void setMode(int posKnee, int posHip);
//funzione per scrivere messaggi ad EXO APP
void writeBluetooth(String message);
//funzione per leggere stringhe da EXO APP 
String readMessageBluetooth();
//funzione per print info su mode corrente
void logStatus(int level);
// funzione per calibrare posizione con joystick
void calibrationMode();
//funzione per resettare valori per curve camminata
void resetValues(float *scale, unsigned int *tempo, float *xT);
*/


/// @brief mode that disable motors 
void freeMode(){ 
    disableMotor(1);
    disableMotor(2);
}

//funzione per resettare valori iniziali variabili curve
void resetValues(float *scale, unsigned int *tempo, float *xT){
  *tempo = 0;
  *xT = 0;
  *scale = 0.0;
}


/// @brief mode with the use of joystick
void joystickMode(byte direction){
  if(direction==0){
    //va avanti con impostazione scale definita negli altri due casi
  }
  else if(direction == 1)
  {
    //avanti
    scala += 0.001;
    
  }
  else if(direction == 2)
  {
    //indietro
    if(scala > 0) scala -= 0.001;
    
  }
  
  tempo = 1.5*xT;
  xT++;
  scaleHori = 0.08;

  sendToMotor(1,getCurveAnca(tempo ,scala, scaleHori),10,2,2,2);
  sendToMotor(2,getCurveGinocchio(tempo ,scala, scaleHori),10,2,2,2);

  Serial.print(getCurveGinocchio(tempo, scala, scaleHori));
  Serial.print(" ");
  Serial.print(getCurveAnca(tempo ,scala, scaleHori));
  delay(10);
}



// mode with gyroscope
void gyroMode(){

  digitalWrite(37,HIGH);
  //ottiene valori da giroscopio
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //inserisce valori gyroscopio all'interno dell'array per utilizzarlo nell'algoritmo di ML
  int data[4];
  data[0] = a.acceleration.x;
  data[1] = a.acceleration.y;
  data[2] = g.gyro.x; 
  Serial.print("Data ML : ");
  Serial.print(ml_calculator.motion(data));
  Serial.print("  ");

  //calcolo del fattore c Machine Learning
  if(ml_calculator.motion(data))
  {
    
    //abilità motori solo prima volta che entra dentro al ciclo, ogni volta che riparte da zero 
    if(xT == 0){
      enterMotorMode(1);
      enterMotorMode(2);
    }

    //muovi motori secondo curva sinosoidale
    //aggiorna valori scala e tempo
    scala += 0.001;
    tempo = 1.5*xT;
    scaleHori = 0.08;
    //print delle curve
    Serial.print(getCurveGinocchio(tempo, scala, scaleHori));
    Serial.print(" ");
    Serial.print(getCurveAnca(tempo, scala, scaleHori));
    Serial.print(" ");
    //send position to the motors 
    sendToMotor(1,getCurveAnca(tempo, scala, scaleHori),10,2,2,2);
    sendToMotor(2,getCurveGinocchio(tempo, scala, scaleHori),10,2,2,2);

    //Serial.print(startPosition(xT));
    //incrementa x della funzione
    xT++;
  }
  else
  {
    // return to home position        
    if(getCurveAnca(tempo, scala, scaleHori) != 0) sendToMotor(1,0,5,2,2,2); // hip
    if(getCurveGinocchio(tempo, scala, scaleHori) != 0) sendToMotor(2,0,5,2,2,2); // knee

    //read values from Motors
    
    //disableMotor(1);
    //disableMotor(2);
    
    //funzione che resetta valori delle curve
    resetValues(&scala, &tempo, &xT);
    
    //due strade da prendere :
    //1) disableMotor e una volta che c soddisfa condizione vera impostare 0 motor nella current pos; 
    //2) quando c non soddisfa condizioni riportare motori su valore 0 preimpostato con una velocità ridotta per poi disabilitare motori
    //TODO forse meglio prima strada per fattore comodità dal momento che una volta disabilitati i motori è possibile fare movimento libero
  }
}



/// @brief mode to check the mode via bluetooth
/// @param mode current mode
void checkModeBluetooth(byte mode){  
  if(mode == 1) freeMode();
  if(mode == 2) gyroMode();
  if(mode == 3); //;
}


// controllo motori con PID !!probabile non serva!!
void PIDmode(int posHip, int posKnee){  
  //TODO
  PidControl pidHip;
  PidControl pidKnee;

  //calcolo PID
  long currT = micros();
  int speedKnee = pidKnee.calculate(currT, knee, posKnee); 
  int speedHip = pidHip.calculate(currT, hip, posHip);

  //DA RIVEDERE 
  //controllo pid utilizzato in gyroMode, quando si stoppa x torna a 0 e...
}


/// @brief convert speed and mode into a message to send to App 
void writeBluetooth(){
  //int messageToSend = (speed << 3) /*+ mode*/ ;
}

/// @brief function to get the Mode selected in the App
/// @param message message come from the App
/// @return current mode
int getModeApp(String message){
  int32_t n = message.toInt();

  return n - ((n >> 3) << 3);
}

/// @brief function to get the Speed selected in App
/// @return 
int getSpeedApp(String message){
  int32_t n = message.toInt();
  
  return ((n - (n >> 9) << 9 )) - (getModeApp(message) >> 3);

}




/// @brief function to readMessage from Serial and put values into variables
void readMessageBluetooth(){
  if(Serial8.available()){
    String message = Serial2.read(); // valore letto da app
    
    modeBluetooth = getModeApp(message);
    speedBluetooth = getSpeedApp(message);
  }


}


//print value in serial
/// @brief Log that print useful information on monitor
/// @param level 0 = controller, 1 = APP mode
void logStatus(int level){ 
  //use of Controller
  if(level == 0){
    Serial.print("    //   ");
    Serial.print(ct.getMode());
    Serial.print("  X direction : ");
    Serial.print(ct.getDirectionX());
    Serial.print("  Y direction : ");
    Serial.print(ct.getDirectionY());
    
  } 
  //mode with APP
  if(level == 1){
    Serial.print("APP>>");
    Serial.print(" Mode: ");
    Serial.print(modeBluetooth);
    Serial.print(" , Speed: ");
    Serial.print(speedBluetooth);

  }

  Serial.println(" ");
  //TODO
}


// function to calibrate initial position
void calibrationMode(){
  int current_ID;
  read_fromMotor(current_ID,position,speed,torque);
  //se id che riceve da messaggio CAN è 1 aggiorna pos motore 1 
  if(current_ID == 1){
    if(ct.getDirectionX()){
      sendToMotor(current_ID,position+1,10,0,0,0); // ultimi 4 valori da testare
    }
    else if(ct.getDirectionX()==2){
      sendToMotor(current_ID,position-1,10,0,0,0); // ultimi 4 valori da testare
    }else{
      //nulla sta fermo
    }
  }

  //se id che riceve da messaggio CAN è 2 aggiorna pos motore 2 
  if(current_ID == 2){
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


/// @brief mode to set the 0 position USE in INTERRUPT
void resetZeroPositionMotor(){
  //insert 0 position only if the current mode is calibrationMode
  if(ct.getMode() == 5){
    setZeroMotor(1);
    setZeroMotor(2);
  }
}

/// @brief mode that set the initial parameter every time mode changed
void transictionMode(){
  //when the button mode is clicked, check if the current mode is 1(gyroMode) and reset the values for the 2 mode(Joystick)
  //if(ct.getMode() == 1){
    resetValues(&scala, &tempo, &xT);
  //}
}


///////////////////////////////////////////////END///////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  //parte MPU 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  //setting gyroscope and accelerometer 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("");
  delay(100);

  //TODO define pin led controller 
  //define serial
  
  Serial8.begin(9600); // --> serial per bluetooth 
  initDriverBUS();
  //define pin bluetooth
  //define other things
  
  //interrupt when button Joystick is clicked, it is used to set the 0 position
  pinMode(j_bt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(j_bt), resetZeroPositionMotor, CHANGE);

  //interrupt when button Mode is clicked, it is used to set the initial parameter for the next Mode
  pinMode(bt2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(bt1), transictionMode, CHANGE);

  pinMode(PIN_BLUETOOTH,INPUT); // declaration of PIN BLUETOOTH

  pinMode(37,OUTPUT);//pin scritta EXO
}


void loop() {
  //if controllo bluetooth
  if(digitalRead(PIN_BLUETOOTH)){ //pin bluetooth attivo --> vuol dire che telefono collegato

    //this function read message from Serial and write the values in variables : modeBluetooth and speedBluetooth; 
    readMessageBluetooth();

    if(modeBluetooth == 1) freeMode(); //mode with motors disable
    if(modeBluetooth == 2) gyroMode(); //attraverso ML attiva e disattiva motori e curva 
    if(modeBluetooth == 3) joystickMode(ct.getDirectionY()); // utilizza joystick per muovere EXO 
    if(modeBluetooth == 5) calibrationMode(); // attraverso controller calibra i motori
    
    logStatus(1);

  }else{
    //Mode without APP 
    if(ct.getMode() == 1) freeMode(); //mode with motors disable
    if(ct.getMode() == 2) gyroMode(); //attraverso ML attiva e disattiva motori e curva 
    if(ct.getMode() == 3) joystickMode(ct.getDirectionY()); // utilizza joystick per muovere EXO 
    if(ct.getMode() == 5) calibrationMode(); // attraverso controller calibra i motori

    ct.checkModeController();
    
    logStatus(0);
  }

}

      
