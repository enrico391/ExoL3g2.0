//classe PID
#include <Arduino.h>

class PidControl {
  private:
    float kp = 1; //costante proportional
    float kd = 0.020; //costante derivate
    float ki = 0;//costante integral
    long prevT = 0;
    float eprev = 0;
    float eintegral = 0;
    int e = 0;
    float dedt;

  public:
    float u ;

    int calculate(long currT, int target, int currentPos ) {
      float deltaT = ((float)(currT - prevT)) / 1.0e6;
      prevT = currT;
      e = currentPos - target;
      dedt = (e - eprev) / (deltaT);
      eintegral = eintegral + e * deltaT;
      //control signal
      u = kp * e + kd * dedt + ki * eintegral;
      eprev = e;
      return u;
    }
};
