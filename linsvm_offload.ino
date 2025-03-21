//This code was autogenerated using micro-learn.
//Use the dummy variable array 'data' to interact with the ML inference.

double w[] = {1.2996878342734843, 0.09082030487999183, 0.2738500377783939};
double u[] = {-2.137123287671233, 0.05847358121330724, 9.215812133072406};
double p[] = {0.2834320056785879, 2.454550912042887, 0.8410172775532369};

double c = -0.04289225225701178;

void setup() {
    Serial.begin(9600);

}

void loop() {
    //Data Section: To Be Coded Manually

    float data[3]; //This is your feature vector. Retrive your data into this array.

    //ML Inference Section

    double temp = 0.0;
    for(int i=0; i<3; i++)
    {
        temp += (data[i]-u[i]) * p[i] * w[i];
    }

    if(temp >= c)
    {
        //Do something for class label 1.
        Serial.println("1");
    }
    else
    {
        //Do something for class label 0.
        Serial.println("0"); 
    }

    delay(1000);
}