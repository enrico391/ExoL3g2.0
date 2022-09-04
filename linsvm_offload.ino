//This code was autogenerated using micro-learn.
//Use the dummy variable array 'data' to interact with the ML inference.

double w[] = {-1.5150847909302836, 0.058820816356530164, -1.1350316251028203, -0.2881538602105478};
double u[] = {-535.2365315546434, -1.0615700359158542, 12.096459722934839, 0.9907644946126218};
double p[] = {0.0007629927776745341, 0.005105650876470956, 0.0008388574130320697, 0.007478236073259843};

double c = 0.525712033288551;

void setup() {
    Serial.begin(9600);

}

void loop() {
    //Data Section: To Be Coded Manually

    float data[4]; //This is your feature vector. Retrive your data into this array.

    //ML Inference Section

    double temp = 0.0;
    for(int i=0; i<4; i++)
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