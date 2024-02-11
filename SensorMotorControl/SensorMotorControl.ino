//Define DAC and ADC pins
#define DAC_CH1 25
#define DAC_CH2 26
#define ADC1_CH4 32
#define ADC1_CH5 33
#define ADC1_CH6 34
#define ADC1_CH7 35 //unused

//past values for filtering
double yn1 = 0;
double yn2 = 0;
double yn3 = 0;
double xn1 = 0;
double xn2 = 0;
double xn3 = 0;

double sensorOutput = 0; //force sensor reading

//filter coefficients
double b[5] = {0.213190813596054,   0.639572440777059,   0.639572440812586 ,  0.213190813584951};
double a[5] = {1.000000000000000,  -2.918420946242358 ,  2.842919157868234,  -0.924327658974999};

//software limit
double rpmLimit = 2000; //ADC value

//error PID terms
double currentError = 0; //for P term
double cumulativeError = 0; //for I term
double previousError = 0; //for D term
double derivativeError = 0; //for D term

//error constants
double Kp = 0.006;
double Ki = 0.000001;
double Kd = 0.000001;

double idealForce = 0; //set force value
double motorCurrent = 0; //DAC value 

int safety = 1; //safety latch for when RPM goes over limit. Need to reset esp32 to reset

unsigned long savedTime = 0;
unsigned long currentTime = 0;

void setup() {
  // put your setup code here, to run once:
  // Setup Serial Monitor
  Serial.begin(115200);

  //scaleing num (b) down by 1e-0x
  for (int i = 0; i < (sizeof(b)/sizeof(b[0])); i++) {
    b[i] = b[i]*0.0001;
  }
  
  savedTime = millis();
}

void loop() {

//set ideal force
  //Events set to occur every 10 ms
  if ((millis()-savedTime)%10==0){
    currentTime = millis();
    //insert code to set force 
    
    if ((millis()-savedTime)<=5000){
      idealForce = 0; //Ideal sensor force, what the sensor should be reading
    }
    else if ((millis()-savedTime)<=10000){
      idealForce = 1000; //Ideal sensor force, what the sensor should be reading
    }
    else if ((millis()-savedTime)<=15000){
      idealForce = 1500; //Ideal sensor force, what the sensor should be reading
    }
    else if ((millis()-savedTime)<=20000){
      idealForce = 1750; //Ideal sensor force, what the sensor should be reading
    }
    else if ((millis()-savedTime)<=25000){
      idealForce = 1000; //Ideal sensor force, what the sensor should be reading
    }
    else {
      idealForce = 0; //Ideal sensor force, what the sensor should be reading
    }
    //Serial.print("Ideal Force: ");
    Serial.print(idealForce);
    Serial.print(",");

//Sensor Read
     double xn = analogRead(ADC1_CH5);
    //output value
    //4th order difference equation - based on matlab filter order
    //double yn = -a[1]*yn1 - a[2]*yn2 - a[3]*yn3 - a[4]*yn4 + b[0]*xn + b[1]*xn1 + b[2]*xn2 + b[3]*xn3 + b[4]*xn4;
    //3rd order difference equation - based on matlab filter order
    double yn = -a[1]*yn1 - a[2]*yn2 - a[3]*yn3 + b[0]*xn + b[1]*xn1 + b[2]*xn2 + b[3]*xn3;

    //saving past values for the difference equation
    yn3 = yn2;
    yn2 = yn1;
    yn1 = yn;

    xn3 = xn2;
    xn2 = xn1;
    xn1 = xn;

    //print statements for serial plotter
    // Serial.print(xn);
    // Serial.print(",");

    //preventing output from going below zero, probably useful to prevent the motor from going opposite direction
    if (yn>=0){
      sensorOutput=yn;
    }
    else if (yn<0){
      sensorOutput = 0;
    }
    Serial.print(sensorOutput);
    Serial.print(",");


//Motor control
    
    // Read analog RPM values from Motor Controller
    //Serial.print("Digital RPM Value = ");
    int rpmRead = analogRead(ADC1_CH6);
    // Serial.print("RPM Read Value: ");
    // Serial.println(rpmRead);
  
    // Read analog current values from Motor Controller
    //Serial.print("Digital Current Value = ");
    int currentRead = analogRead(ADC1_CH4);
    // Serial.print("Current Read Value: ");
    //Serial.println(currentRead);

    //only run of RPM is within bounds
    
   if ((rpmRead <= rpmLimit) && (rpmRead >=-rpmLimit) && (safety ==1)) { //1000 placeholder for digital RPM equivilent of chosen limit RPM
      analogWrite(DAC_CH2,int(motorCurrent));
      Serial.println(int(motorCurrent));   
    }
    else {
      analogWrite(DAC_CH2,0);
      Serial.println("SAFETY");  
      safety = 0;
    }

    if ((sensorOutput<=500) && (currentRead>=1000)) { //if sensor feels zero, stop giving current. Used to prevent the addition of force when finger isnt on properly.
      safety = 0;
    }



  
//get Error
    previousError = currentError; //saves previous error
    currentError = (idealForce - sensorOutput)/4095*255; 
    cumulativeError = cumulativeError + currentError; //get total error
    derivativeError = (currentError - previousError) / (millis()-currentTime);
    
    //apply error and limit motorCurrent to [0,255]

    if (((motorCurrent + Kp* currentError + Ki * cumulativeError + Kd * derivativeError) <=255) && ((motorCurrent + Kp* currentError + Ki * cumulativeError + Kd * derivativeError) >=0)) {
    motorCurrent = motorCurrent + Kp* currentError + Ki * cumulativeError;
    }
    else if (motorCurrent + Kp* currentError + Ki * cumulativeError + Kd * derivativeError >255){
      motorCurrent = 255;
    }
    else if (motorCurrent + Kp* currentError + Ki * cumulativeError + Kd * derivativeError<0){
      motorCurrent = 0;

    }
  }
  
}