#define PIN_BLUETOOTH 33
#include <iostream>
#include <Arduino.h>
//#include <SPI.h>
//#include <mcp2515.h>
#include <tMotor.h>
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

float setPosHip;
float setPosKnee;

long unsigned int current_ID;


float id;

//oggetto controller per ricevere info su controller mode
Controller ct;

//variables for App control
byte modeBluetooth;
int speedBluetooth;


//variable to start motor every time the mode is changed in 1 
bool startMotor = true;

//message from APP
String messageB;


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

void gravityMode(){
  read_fromMotor(current_ID,position,speed,torque);
  float p_in = position;
  float t_in = 10 * sin(position + 0.06) + 0.0;
  Serial.println(torque);
  sendToMotor(current_ID,p_in,0,1,2,t_in);
}


//function to read values of motor
float readMotorValues(){
  read_fromMotor(current_ID,position,speed,torque);

  if(current_ID == 1){
    Serial.print("Posizione motore Anca : ");
    Serial.print(position);
    Serial.print("  Speed motor Anca : ");
    Serial.print(speed);
    Serial.print("  Torque motore Anca : ");
    Serial.print(torque);
  }
  if(current_ID == 2){
    Serial.print("Posizione motore Ginocchio : ");
    Serial.print(position);
    Serial.print("  Speed motor Ginocchio : ");
    Serial.print(speed);
    Serial.print("  Torque motore Ginocchio : ");
    Serial.print(torque);
  }
  else {
    Serial.print("ID : "); Serial.print(current_ID); Serial.print("  Posizione : "); 
    Serial.print(position);
    Serial.print("  Speed motor Ginocchio : ");
    Serial.print(speed);
    Serial.print("  Torque motore Ginocchio : ");
    Serial.print(torque);
  }


  
}

float degreeToRadiant(float degree){
  return degree*(PI/180);
}


/// @brief mode that disable motors 
void freeMode(){ 
    //disableMotor(1);
    //disableMotor(2);
    if(startMotor){
      enterMotorMode(0x001); //hip 
      enterMotorMode(0x002); //knee 

      startMotor = false;
    }
    

    
    sendToMotor(0x001, 0, 0, 2, 0.8, 0);  
    sendToMotor(0x002, 0, 0, 2, 0.8, 0); 
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
    scala += 0.01;
    
  }
  else if(direction == 2)
  {
    //indietro
    if(scala > 0) scala -= 0.01;
    
  }
  
  tempo = 1.5*xT;
  xT++;
  scaleHori = 0.08;

  sendToMotor(0x001,2*degreeToRadiant(getCurveAnca(tempo ,scala, scaleHori)),0, 2, 1, 0); 
  sendToMotor(0x002,2*degreeToRadiant(getCurveGinocchio(tempo ,scala, scaleHori)),0, 2, 1, 0); 

  Serial.print(2* degreeToRadiant(getCurveGinocchio(tempo, scala, scaleHori)));
  Serial.print(" ");
  Serial.print(2* degreeToRadiant(getCurveAnca(tempo ,scala, scaleHori)));
  delay(10);
}



// mode with gyroscope
void gyroMode(){
  //pin LED
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
      
      enterMotorMode(0x001);
      enterMotorMode(0x002);

      sendToMotor(0x001, 0, 0, 1, 1.2, 0); 
      sendToMotor(0x002, 0, 0, 1, 1.2, 0); 
    }

    //muovi motori secondo curva sinosoidale
    //aggiorna valori scala e tempo
    scala += 0.001;
    tempo = 1.5*xT;
    scaleHori = 0.08;
    //print delle curve
    Serial.print(2*degreeToRadiant(getCurveGinocchio(tempo, scala, scaleHori)));
    Serial.print(" ");
    Serial.print(2*degreeToRadiant(getCurveAnca(tempo, scala, scaleHori)));
    Serial.print(" ");
    //send position to the motors 
    sendToMotor(0x001, (2*degreeToRadiant(getCurveAnca(tempo, scala, scaleHori))), 0, 2, 1, 0); 
    sendToMotor(0x002, (2*degreeToRadiant(getCurveGinocchio(tempo, scala, scaleHori))), 0, 2, 1, 0);  // 0, 2, 1, 0); 


    //Serial.print(startPosition(xT));
    //incrementa x della funzione
    
    xT++;
  }
  else
  {
    // return to home position        
    //if(getCurveAnca(tempo, scala, scaleHori) != 0) sendToMotor(0x001, 0, 1, 1, 1, 1); // hip
    //if(getCurveGinocchio(tempo, scala, scaleHori) != 0) sendToMotor(0x002, 0, 0.5, 0.5, 0.5, 0.5); // knee


    sendToMotor(0x001, 0, 0, 1, 1.2, 0); 
    sendToMotor(0x002, 0, 0, 1, 1.2, 0); 
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
    messageB = Serial8.read(); // valore letto da app
    modeBluetooth = getModeApp(messageB);
    speedBluetooth = getSpeedApp(messageB);


    //reset all variables 
    startMotor = true; // --> for start motor in 1 mode after mode 5  (mode 5 to 1 enable motors)
    //funzione che resetta valori delle curve
    resetValues(&scala, &tempo, &xT); 
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
    Serial.print(messageB);
    Serial.print(" Mode: ");
    Serial.print(modeBluetooth);
    Serial.print(" , Speed: ");
    Serial.print(speedBluetooth);

  }

  //calibration mode
  if(level == 2){
  }

  Serial.println(" ");
  //TODO
}


// function to calibrate initial position
void calibrationMode(){
  if(ct.getMode() == 5 || modeBluetooth == 5 ){
    //ct.readJoystick(setPosHip,setPosKnee);
    //sendToMotor(0x001, degreeToRadiant(setPosHip),0, 2, 1, 0); // ultimi 4 valori da testare
    //sendToMotor(0x002, degreeToRadiant(setPosKnee), 0, 2, 1, 0);  // ultimi 4 valori da testare

    exitMotorMode(0x001);
    exitMotorMode(0x002);

  }
}


/// @brief mode to set the 0 position USE in INTERRUPT
void resetZeroPositionMotor(){
  //insert 0 position only if the current mode is calibrationMode
  if(ct.getMode() == 5){
    /*
    resetValues(&scala, &tempo, &xT);
    //sendToMotor(0x001, degreeToRadiant(setPosHip),0, 2, 1, 0); // ultimi 4 valori da testare
    //sendToMotor(0x002, degreeToRadiant(setPosKnee), 0, 2, 1, 0);  // ultimi 4 valori da testare
    //delay(100);
    enterMotorMode(0x001);
    enterMotorMode(0x002);
    setZeroMotor(0x001);
    setZeroMotor(0x002);
    //imposta mode 1 e manageLED 1
    setPosHip = 0;
    setPosKnee = 0;
    ct.setMode(1);
    ct.manageLED(1);
    */
  }
}

/// @brief mode that set the initial parameter every time mode changed INTERRUPT
void transictionMode(){
  //when the button mode is clicked, check if the current mode is 1(gyroMode) and reset the values for the 2 mode(Joystick)
  //if(ct.getMode() == 1){
    resetValues(&scala, &tempo, &xT);

    //condition to avoid click of motors when mode change between 1 and 2
    if(ct.getMode() != 1){
      sendToMotor(0x001,0,0, 1, 1.2, 0); 
      sendToMotor(0x002,0,0, 1, 1.2, 0); 
    }
}


/// @brief mode that set the initial parameter every mode calibration is called  INTERRUPT
void transictionSetMode(){
  //when the button mode is clicked, check if the current mode is 1(gyroMode) and reset the values for the 2 mode(Joystick)
  //if(ct.getMode() == 1){
    resetValues(&scala, &tempo, &xT);

    //condition to avoid click of motors when mode change between 1 and 2
    sendToMotor(0x001,0,0, 1, 1.2, 0); 
    sendToMotor(0x002,0,0, 1, 1.2, 0); 

    //reset value FLAG for enable motors every time mode is changed in 1 
    startMotor = true; // --> for start motor in 1 mode after mode 5  (mode 5 to 1 enable motors)
}














///////////////////////////////////////////////END///////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial8.begin(9600); // --> serial per bluetooth 

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
  
  

  //init CAN shield
  checkCAN();
  
  
  //interrupt when button Joystick is clicked, it is used to set the 0 position
  pinMode(j_bt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(j_bt), resetZeroPositionMotor, CHANGE);

  //interrupt when button Mode is clicked, it is used to set the initial parameter for the next Mode
  pinMode(bt2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(bt1), transictionMode, CHANGE);

  pinMode(bt1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(bt1), transictionSetMode, CHANGE);

  pinMode(PIN_BLUETOOTH,INPUT); // declaration of PIN BLUETOOTH

  pinMode(37,OUTPUT);//pin scritta EXO


  enterMotorMode(0x001); //hip
  enterMotorMode(0x002); //knee

  //set a first calibration
  setZeroMotor(0x001);
  setZeroMotor(0x002);
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

  }
  
  else{
    
    //Mode without APP 
    if(ct.getMode() == 1) freeMode(); //mode with motors disable
    if(ct.getMode() == 2) gyroMode(); //attraverso ML attiva e disattiva motori e curva 
    if(ct.getMode() == 3) joystickMode(ct.getDirectionY()); // utilizza joystick per muovere EXO 
    if(ct.getMode() == 5) {
      calibrationMode(); // attraverso controller calibra i motori
      logStatus(2);
    }

    ct.checkModeController();
    
  }

  //readMotorValues();


  Serial.println();

}

      
