#include <Arduino.h>

class Controller{
    private:
      #define LED1 9
      #define LED2 11
      #define LED3 12
      #define bt1 2
      #define bt2 3
      #define j_y 15 //analogPin
      #define j_x 16 //analogPin
      #define j_bt 4

      int val_x, val_y, angleX, angleY;
      byte mode = 1;


    public:
      /// @brief costructor of controller 
      Controller(){
        pinMode(bt1, INPUT_PULLUP);
        pinMode(bt2, INPUT_PULLUP);
        pinMode(LED1,OUTPUT);
        pinMode(LED2,OUTPUT);
        pinMode(LED3,OUTPUT);
        
        manageLED(1);
      }

      /// @brief method for update current Mode
      void checkModeController() {
        if(digitalRead(bt1) == LOW){
          //aggiorna variabile mode
          mode++;
          //se mode >= 4 ritorna alla prima mode
          if(mode>=4) mode = 1;
          //richiama metodo per accendere led
          manageLED(mode);
          delay(2000);
        }
        //if all 2 button is pressed update the mode in 5
        if(digitalRead(bt2) == LOW ){
          mode = 5;
          manageLED(mode);
        }
      }

      /// @brief getCurrentMode
      /// @return currentMode
      inline int getMode(){
        return mode;
      }


      /// @brief update the led in controller
      /// @param mode 
      void manageLED(byte mode){
        
        if(mode == 1){
          digitalWrite(LED1,HIGH);
          digitalWrite(LED2,LOW);
          digitalWrite(LED3,LOW);
        }
        
        if(mode == 2){
          digitalWrite(LED1,LOW);
          digitalWrite(LED2,HIGH);
          digitalWrite(LED3,LOW);
        }
        if(mode == 3){
          digitalWrite(LED1,LOW);
          digitalWrite(LED2,LOW);
          digitalWrite(LED3,HIGH);
        }
        if(mode == 5){
          digitalWrite(LED1,HIGH);
          digitalWrite(LED2,HIGH);
          digitalWrite(LED3,HIGH);
        }
      }

      /// @brief print variable of controller 
      /// @param mode 
      void log(byte mode){
        Serial.print("Mode : ");
        Serial.print(mode);
        Serial.print(" Val x : ");
        Serial.print(val_x);
        Serial.print(" Val y : ");
        Serial.print(val_y);
        Serial.print(" AngleX : ");
        Serial.print(angleX);
        Serial.print(" AngleY : ");
        Serial.println(angleY);
      }

      /// @brief read the values from joystick and update the variable passed by address
      /// @param angleX angle of motor hip
      /// @param angleY angle of motor knee
      void readJoystick(int *angleX, int *angleY){
        //read analog value in ESP is between 0 to 4095
        int val_x = analogRead(j_x);
        int val_y = analogRead(j_y);

        //map the values
        val_x = map(val_x,0,1024,-500,500);
        val_y = map(val_y,0,1024,-500,500);

        //cast the numbers
        if(val_x>-200 && val_x<200){
          val_x = 0;
        }
        if(val_y>-200 && val_y<200){
          val_y = 0;
        }

        //update angles
        if(val_x>0){
          *angleX++;
        }else if(val_x<0){
          *angleX--;
        }

        //update angles 
        if(val_y>0){
          *angleY++;
        }else if(val_y<0){
          *angleY--;
        }
      }

      /// @brief return the direction of joystick
      /// @return 1 forward, 2 backward, 0 stop
      int getDirectionY(){
        int val_y = analogRead(j_y);
        val_y = map(val_y,0,1024,-500,500);
        //cast value
        if(val_y>-200 && val_y<200){
          val_y = 0;
        }
      
        if(val_y>200){
          return 1;
        }
        else if(val_y<-200){
          return 2;
        }
        else{
          return 0;
        }
      }

      /// @brief return the direction of joystick
      /// @return 1 forward, 2 backward, 0 stop
      int getDirectionX(){
        int val_x = analogRead(j_x);
        val_x = map(val_x,0,1024,-500,500);

        if(val_x>-200 && val_x<200){
          val_x = 0;
        }

        if(val_x>200){
          return 1;
        }else if(val_x<-200){
          return 2;
        }else{
          return 0;
        }
      }
};

