#include <Arduino.h>


class Svm{
    private :
        double w[4] = {-1.5150847909302836, 0.058820816356530164, -1.1350316251028203, -0.2881538602105478};
        double u[4] = {-535.2365315546434, -1.0615700359158542, 12.096459722934839, 0.9907644946126218};
        double p[4] = {0.0007629927776745341, 0.005105650876470956, 0.0008388574130320697, 0.007478236073259843};
        double c = 0.525712033288551;
        //controlla array indice !!!!!!!

    public:
        bool motion(int data[]){
            double temp = 0.0;
            for(int i=0; i<4; i++)
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



