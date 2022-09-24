//#include <Arduino.h>

//buoni con retta
//float a[3] = {6.964,-13.83,1.111};
//float b[2] = {14.09,-5.848};
//float w = 0.04695;

//coefficienti curva ANCA
float a[3] = {6.964,-13.83,1.111};
float b[2] = {14.09,-5.848};
float w = 0.04695;
//float curvaAnca = a[0] + a[1]*cos(time*w)+ b[0]*sin(time*w) + a[2]*cos(2*time*w) + b[1]*sin(2*time*w);

//coefficienti curva GINOCCHIO
float a_g[3] = {-30.2,-21.92,  10.27};
float b_g[2] = {-12.73,-20.42 };
float w_g =0.04675 ;
//float curvaGinocchio = a_g[0] + a_g[1]*cos(time*w_g)+ b_g[0]*sin(time*w_g) + a_g[2]*cos(2*time*w_g) + b_g[1]*sin(2*time*w_g);





// @brief get the value of angle of hip with the curve
// @param time the x value of the function curve
// @param scale the scale factor for start with a small curve
// @return angle for control motor
float getCurveAnca( float time,float scale, float scaleHori){

    if(scale<=1){
        return scale*(a[0] + a[1]*cos((scaleHori*time)*w)+ b[0]*sin((scaleHori*time)*w) + a[2]*cos(2*(scaleHori*time)*w) + b[1]*sin(2*(scaleHori*time)*w));
    }else {
        return a[0] + a[1]*cos((scaleHori*time)*w)+ b[0]*sin((scaleHori*time)*w) + a[2]*cos(2*(scaleHori*time)*w) + b[1]*sin(2*(scaleHori*time)*w);
    }
}



/// @brief get the value of angle of knee with the curve
/// @param time the x value of the function curve
/// @param scale the scale factor for start with a small curve it equals of the range of walking
/// @return angle for control motor
float getCurveGinocchio(float time, float scale, float scaleHori){
    if(scale<=1){
        return scale*(a_g[0] + a_g[1]*cos((scaleHori*time)*w_g)+ b_g[0]*sin((scaleHori*time)*w_g) + a_g[2]*cos(2*(scaleHori*time)*w_g) + b_g[1]*sin(2*(scaleHori*time)*w_g));
    }else{
        return (a_g[0] + a_g[1]*cos((scaleHori*time)*w_g)+ b_g[0]*sin((scaleHori*time)*w_g) + a_g[2]*cos(2*(scaleHori*time)*w_g) + b_g[1]*sin(2*(scaleHori*time)*w_g));
    }}



/// @brief find the x position to start walking
/// @param y the current pos of motor
/// @return x value(time)
float startPosition(float y, float scaleHori){
    //y = scale*(a[0] + a[1]*cos((scaleHori*time)*w)+ b[0]*sin((scaleHori*time)*w) + a[2]*cos(2*(scaleHori*time)*w) + b[1]*sin(2*(scaleHori*time)*w));
    
    return (a[0] + a[1]*acos((scaleHori*y)*w)+ b[0]*asin((scaleHori*y)*w) + a[2]*acos(2*(scaleHori*y)*w) + b[1]*asin(2*(scaleHori*y)*w));
}


