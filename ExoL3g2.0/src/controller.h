#include <Arduino.h>

class Controller{
    private:
      #define LED1 8
      #define LED2 9
      #define LED3 10
      #define bt1 11
      #define bt2 12
      #define j_x A0
      #define j_y A1
      #define j_bt 13

      int val_x, val_y, angleX, angleY;
      byte mode = 1;


    public:

      Controller(){
        pinMode(bt1, INPUT_PULLUP);
        pinMode(bt2, INPUT_PULLUP);
        pinMode(LED1,OUTPUT);
        pinMode(LED2,OUTPUT);
        pinMode(LED3,OUTPUT);
        manageLED(1);
      }
      void checkModeController() {

        if(digitalRead(bt1) == LOW){
        mode++;
        if(mode>=4){
          mode = 1;
        }
        manageLED(mode);
        delay(1000);
      }
      if(digitalRead(bt2) == LOW && digitalRead(bt1) == LOW){
        mode = 5;
      }
      }


      int getMode(){
        return mode;
      }

    void manageLED(byte mode){
      if(mode == 1){
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,LOW);
      }
      else if(mode == 2){
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,LOW);
      }
      else if(mode == 3){
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,HIGH);
      }
      else if(mode == 5){
        digitalWrite(LED1,HIGH);
        digitalWrite(LED2,HIGH);
        digitalWrite(LED3,HIGH);
      }

    }

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

    void readJoystick(int& angleX, int& angleY){
        int val_x = analogRead(j_x);
        int val_y = analogRead(j_y);
        val_x = map(val_x,0,1023,-500,500);
        val_y = map(val_y,0,1023,-500,500);
        if(val_x>-30 && val_x<30){
          val_x = 0;
        }
        if(val_y>-30 && val_y<30){
          val_y = 0;
        }

        if(val_x>0){
          angleX++;
        }else if(val_x<0){
          angleX--;
        }else{

        }
        if(val_y>0){
          angleY++;
        }else if(val_y<0){
          angleY--;
        }else{

        }
    }
    int getDirectionX(){
        int val_x = analogRead(j_x);
        val_x = map(val_x,0,1023,-500,500);
        if(val_x>-30 && val_x<30){
          val_x = 0;
        }
        
        if(val_x>0){
          return 1;
        }else if(val_x<0){
          return 2;
        }else{
          return 0;
        }
    }
    int getDirectionY(){
        int val_y = analogRead(j_y);
        val_y = map(val_y,0,1023,-500,500);
        if(val_y>-30 && val_y<30){
          val_y = 0;
        }
        if(val_y>0){
          return 1;
        }else if(val_y<0){
          return 2;
        }else{
          return 0;
        }
    }
};

