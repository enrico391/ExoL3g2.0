#include <Arduino.h>

class Svm{
    
    private :
        double w[3] = {1.2996878342734843, 0.09082030487999183, 0.2738500377783939};
        double u[3] = {-2.137123287671233, 0.05847358121330724, 9.215812133072406};
        double p[3] = {0.2834320056785879, 2.454550912042887, 0.8410172775532369};
        double c = 0.525712033288551;
        //controlla array indice !!!!!!!

    public:
        /// @brief method to check if values read from Gyro/Accelerometer is checked or not
        /// @param data array of values read from Gyro/Accel
        /// @return true if the check results in true or false 
        bool motion(int data[]){
            double temp = 0.0;
            for(int i=0; i<3; i++)
            {
                temp += (data[i]-u[i]) * p[i] * w[i];
            }

            //Serial.println(temp);
            if(temp >= c)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
};



