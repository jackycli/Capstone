//LCD Menu definitions

#include <ItemInput.h>
#include <ItemList.h>
#include <ItemSubMenu.h>
#include <LcdMenu.h> // Always comes after every item type import

#define LCD_ROWS 4
#define LCD_COLS 20

// Configure keyboard keys (ASCII)
#define UP 56        // NUMPAD 8
#define DOWN 50      // NUMPAD 2
#define ENTER 53     // NUMPAD 5
#define BACK 55      // NUMPAD 7
#define LEFT 52      // NUMPAD 4 DONT NEED prev 52
#define RIGHT 54     // NUMPAD 6 DONT NEED prev 54
#define BACKSPACE 8  // BACKSPACE DONT NEED
#define CLEAR 46     // NUMPAD . DONT NEED

// Set up buttons
#define backbut 27
#define downbut 14
#define enterbut 12
#define upbut 13

// Button state storage
int backlast = HIGH;
int backcurr;
int downlast = HIGH;
int downcurr;
int enterlast = HIGH;
int entercurr;
int uplast = HIGH;
int upcurr;

// RANGE SubMenu
// Declare the array
extern String range[];
// Initialize the array
String range[] = {"1",  "2",  "3", "4", "5"};

// REPETITIONS SubMenu
// Declare the array
extern String repetitions[];
// Initialize the array
String repetitions[] = {"3",  "4",  "5"};

// DURATION SubMenu
// Declare the array
extern String duration[];
// Initialize the array
String duration[] = {"3",  "4",  "5", "6",  "7",  "8"};

// PHASE 1 TEST SubMenu
extern String phase1[];
// Initialize the array
String phase1[] = {"No",  "Yes"};

// PHASE 2 TEST SubMenu
extern String phase2[];
// Initialize the array
String phase2[] = {"No",  "Yes"};

// PHASE 3 TEST SubMenu
extern String phase3[];
// Initialize the array
String phase3[] = {"No",  "Yes"};

// Declare the call back function
void rangeCallback(uint16_t ran);
void repetitionsCallback(uint16_t reps);
void durationCallback(uint16_t dur);
void phase1Callback(uint16_t phas1);
void phase2Callback(uint16_t phas2);
void phase3Callback(uint16_t phas3);

int range_int = 0;
int repetitions_int = 0;
int duration_int = 0;

// prev current counters
int prev_range = 0;


int phase1_int = 0;
int phase2_int = 0;
int phase3_int = 0;

extern MenuItem* rangeMenu[];
extern MenuItem* repetitionsMenu[];
extern MenuItem* durationMenu[];
extern MenuItem* phase1Menu[];
extern MenuItem* phase2Menu[];
extern MenuItem* phase3Menu[];

MAIN_MENU( // DON'T FORGET THE COMMA !!!!!!! 
    ITEM_SUBMENU("Range (N)", rangeMenu), 
    ITEM_SUBMENU("Test Repetitions", repetitionsMenu), 
    ITEM_SUBMENU("Duration", durationMenu),
    ITEM_SUBMENU("Phase 1", phase1Menu),
    ITEM_SUBMENU("Phase 2", phase2Menu),
    ITEM_SUBMENU("Phase 3", phase3Menu)
);

/**
 * Create submenu and precise its parent
 */
SUB_MENU(rangeMenu, mainMenu,   
    ITEM_STRING_LIST("Range (N)", range, 5, rangeCallback)
);

SUB_MENU(repetitionsMenu, mainMenu,
    ITEM_STRING_LIST("# of Reps.", repetitions, 3, repetitionsCallback)
);

SUB_MENU(durationMenu, mainMenu,
    ITEM_STRING_LIST("Duration (s)", duration, 6, durationCallback)
);

SUB_MENU(phase1Menu, mainMenu,
    ITEM_STRING_LIST("Start test?", phase1, 2, phase1Callback)
);

SUB_MENU(phase2Menu, mainMenu,
    ITEM_STRING_LIST("Start test?", phase2, 2, phase2Callback)
);

SUB_MENU(phase3Menu, mainMenu,
    ITEM_STRING_LIST("Start test?", phase3, 2, phase3Callback)
);

LcdMenu menu(LCD_ROWS, LCD_COLS);

// JACKY PART!!!!!!
//motorControl Define DAC and ADC pins
#define MOTORINPUTPIN 33 //used to be 26
#define CURRENTREADPIN 34
#define SENSORREADPIN 35
#define RPMREADPIN 32

//motorControl past values for filtering
double yn1 = 0;
double yn2 = 0;
double yn3 = 0;
double xn1 = 0;
double xn2 = 0;
double xn3 = 0;
double sensorOutput = 0; //force sensor reading

//motorControl filter coefficients
double b[5] = {0.213398512073359,   0.640195536174559,   0.640195536307786 ,  0.213398512030061};
double a[5] = {1.000000000000000,  -2.918393191953221 ,  2.842867652821765,  -0.924303742058885};

//motorControl software limit
double rpmLimit = 3800; //RPM safety ADC value
double currentLimit = 2000; //Current safety ADC value
//motorControl error PID terms
double currentError = 0; //for P term
double cumulativeError = 0; //for I term
double previousError = 0; //for D term
double derivativeError = 0; //for D term

//motorControl error constants
double Kp = 0.010;
double Ki = 0.000001;
double Kd = 0.00001;
//motorControl items
double motorCurrent = 0; //DAC value 
int safety = 1; //safety latch for when RPM or current (special case, see motorcontrol section) goes over limit. Need to reset esp32 to reset

//double idealForceArray[9];

double idealForce = 0; //set force value
unsigned long savedTime1 = 0; //Used for specific timeing for sampling freq
unsigned long savedTime2 = 0;
int lengthForceArray = 0;
int counter1 = 0;

int idealForceArray[9];
int idealForceADC[9];

void setup() {
  // Setup Serial Monitor
   Serial.begin(115200);
   menu.setupLcdWithMenu(0x27, mainMenu);

   // initialize the pushbuttons pin as an pull-up input
   pinMode(backbut, INPUT_PULLUP);
   pinMode(downbut, INPUT_PULLUP);
   pinMode(enterbut, INPUT_PULLUP);
   pinMode(upbut, INPUT_PULLUP);

  //motorControl filter coefficient: scaleing numerator (b) down by 1e-0x
  for (int i = 0; i < (sizeof(b)/sizeof(b[0])); i++) {
    b[i] = b[i]*0.0001;
  }
  
  //save start time
  savedTime1 = millis();
}

void loop() {

    //Serial.println(range_int);
    if(range_int != prev_range){
      //declaring

    if (range_int==1){
      
      double idealForceArray[] = {1, 0, 0, 0, 0, 0, 0, 0, 0};
      
      lengthForceArray= sizeof(idealForceArray)/sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      // for (int i =0; i<lengthForceArray; i++){
      //   Serial.print(idealForceArray[i]);
      //   Serial.print(" ");
      // }
      Serial.println(" ");
      for (int i =0; i<lengthForceArray; i++){
        idealForceADC[i] = 29.9836*pow(idealForceArray[i],2)+595.772*idealForceArray[i]-14.7438;
        // Serial.print(idealForceADC[i]);
        // Serial.print(" ");
      }
    }
    else if (range_int==2){
      double idealForceArray[] = {1, 1.5, 2, 0, 0, 0, 0, 0, 0};
      
      lengthForceArray= sizeof(idealForceArray)/sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      // for (int i =0; i<lengthForceArray; i++){
      //   Serial.print(idealForceArray[i]);
      //   Serial.print(" ");
      // }
      Serial.println(" ");
      for (int i =0; i<lengthForceArray; i++){
        idealForceADC[i] = 29.9836*pow(idealForceArray[i],2)+595.772*idealForceArray[i]-14.7438;
        // Serial.print(idealForceADC[i]);
        // Serial.print(" ");
      }
    }
    else if (range_int==3){
      double idealForceArray[] = {1, 1.5, 2, 2.5, 3, 0, 0, 0, 0};
      
      lengthForceArray= sizeof(idealForceArray)/sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      // for (int i =0; i<lengthForceArray; i++){
      //   Serial.print(idealForceArray[i]);
      //   Serial.print(" ");
      // }
      Serial.println(" ");
      for (int i =0; i<lengthForceArray; i++){
        idealForceADC[i] = 29.9836*pow(idealForceArray[i],2)+595.772*idealForceArray[i]-14.7438;
        // Serial.print(idealForceADC[i]);
        // Serial.print(" ");
      }
    }
    else if (range_int==4){
      double idealForceArray[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 0, 0};
      
      lengthForceArray= sizeof(idealForceArray)/sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      // for (int i =0; i<lengthForceArray; i++){
      //   Serial.print(idealForceArray[i]);
      //   Serial.print(" ");
      // }
      Serial.println(" ");
      for (int i =0; i<lengthForceArray; i++){
        idealForceADC[i] = 29.9836*pow(idealForceArray[i],2)+595.772*idealForceArray[i]-14.7438;
        // Serial.print(idealForceADC[i]);
        // Serial.print(" ");
      }
    }
    else if (range_int==5){
      double idealForceArray[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
      
      lengthForceArray= sizeof(idealForceArray)/sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      // for (int i =0; i<lengthForceArray; i++){
      //   Serial.print(idealForceArray[i]);
      //   Serial.print(" ");
      // }
      Serial.println(" ");
      for (int i =0; i<lengthForceArray; i++){
        idealForceADC[i] = 29.9836*pow(idealForceArray[i],2)+595.772*idealForceArray[i]-14.7438;
        // Serial.print(idealForceADC[i]);
        // Serial.print(" ");
      }
    }
    else{
      Serial.print("Error");
    }

    prev_range = range_int;
    Serial.println(" ");

    savedTime2 = millis();
    counter1 = 0;

    }
  
  
  //sending newton values into ADC and then send to motor control
  if (((millis() - savedTime2) >= 5000) && (lengthForceArray>0)){
    if ((counter1<lengthForceArray) && (idealForceADC[counter1]> 0)){
      idealForce = idealForceADC[counter1];
      counter1++;
    }
    else if ((idealForceADC[counter1] <= 0) && (counter1<lengthForceArray-1)){
        while ((idealForceADC[counter1] <= 0) && (counter1<lengthForceArray-1)){
          counter1++;
        }
        idealForce = idealForceADC[counter1];
        if (counter1<lengthForceArray-1){
          counter1++;
        }
        
      
    }
    else {
      idealForce = 0;
    }

      savedTime2 = millis();
  }

<<<<<<< HEAD
  sensorRead();
  motorControl(idealForce);


  //UI Control Loop
  char command;
  // reading buttons
  backcurr = digitalRead(backbut); // keep in code
  downcurr = digitalRead(downbut);
  entercurr = digitalRead(enterbut);
  upcurr = digitalRead(upbut);
  //Serial.print(backcurr);
  if(backlast == HIGH && backcurr == LOW)
    command = 55;
    // save the last state
  backlast = backcurr;
=======
  
  motorControl(idealForce);

>>>>>>> parent of 92ea32c (Commenting and formatting)


    char command;
    // reading buttons
    backcurr = digitalRead(backbut); // keep in code
    downcurr = digitalRead(downbut);
    entercurr = digitalRead(enterbut);
    upcurr = digitalRead(upbut);
    //Serial.print(backcurr);
    if(backlast == HIGH && backcurr == LOW)
     command = 55;
      // save the last state
    backlast = backcurr;

    if(downlast == HIGH && downcurr == LOW)
      command = 50;
      // save the last state
   downlast = downcurr;   

<<<<<<< HEAD
  switch (command) {
    case UP:
        menu.up();
        menu.right();
        break;
    case DOWN:
        menu.down();
        menu.left();
        break;
    case LEFT:
        menu.left();
        break;
    case RIGHT:
        menu.right();
        break;
    case ENTER:  // Press enter to go to edit mode : for ItemInput
        menu.enter();
  
        break;
    case BACK:
        menu.back();
        break;
    case CLEAR:
        menu.clear();
        break;
    case BACKSPACE:  // Remove one character from tail
        menu.backspace();
        break;
    default:
        break;
  }
=======
    if(enterlast == HIGH && entercurr == LOW)
     command = 53;
      // save the last state
   //enterlast = entercurr;

    if(uplast == HIGH && upcurr == LOW)
     command = 56;
      // save the last state
   uplast = upcurr; 

    switch (command) {
        case UP:
            menu.up();
            menu.right();
            break;
        case DOWN:
            menu.down();
            menu.left();
            break;
        case LEFT:
            menu.left();
            break;
        case RIGHT:
            menu.right();
            break;
        case ENTER:  // Press enter to go to edit mode : for ItemInput
            menu.enter();
      
            break;
        case BACK:
            menu.back();
            break;
        case CLEAR:
            menu.clear();
            break;
        case BACKSPACE:  // Remove one character from tail
            menu.backspace();
            break;
        default:
            break;
    }
>>>>>>> parent of 92ea32c (Commenting and formatting)
  

}

// CALLBACK FUNCTIONS
void rangeCallback(uint16_t ran) {
    // do something with the index
    //Serial.print("String: ");
    //Serial.println(range[ran]);
    
    // convert string found at index 'ran' to int
    range_int = (range[ran]).toInt();
    //Serial.print("Range (N): 0.5-");
    //Serial.println(range_int);
}

void repetitionsCallback(uint16_t reps) {
    // do something with the index
    //Serial.print("String: ");
    //Serial.println(repetitions[reps]);

    // convert string found at index 'reps' to int
    repetitions_int = (repetitions[reps]).toInt();
    //Serial.print("Repetitions: ");
    Serial.println(repetitions_int);
}

void durationCallback(uint16_t dur) {
    // do something with the index
    //Serial.print("String: ");
    //Serial.println(duration[dur]);

    // convert string found at index 'dur' to int
    duration_int = (duration[dur]).toInt();
    //Serial.print("Duration (s): ");
    Serial.println(duration_int);
}

void phase1Callback(uint16_t phas1) {
    // do something with the index
    // if selected "yes" do smth
    
    if (phas1 == 1) { // if 2nd index string is selected (Yes)
      phase1Display(); // hide menu and display new message of current active test phase
      phase1_int = 1;

    } else if (phas1 == 0) { // if 1rst index string is selected (No)
      //menu.show(); // Keep Menu active: don't start phase test
      phase1_int = 0;
    }
    Serial.println(phase1_int);
    //phas1 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 1 SUBMENU
}

void phase1Display() {
  menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 1:");

    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
    // simulate phase 1 ending
    delay(3000);
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("Complete!     ");
    delay(3000);
  menu.show();
}

void phase2Callback(uint16_t phas2) {
    // do something with the index
    // if selected "yes" do smth
    
    if (phas2 == 1) { // if 2nd index string is selected (Yes)
      phase2Display(); // hide menu and display new message of current active test phase
      phase2_int = 1;

    } else if (phas2 == 0) { // if 1rst index string is selected (No)
      //menu.show(); // Keep Menu active: don't start phase test
      phase2_int = 0;
    }
    Serial.println(phase2_int);
    //phas2 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 2 SUBMENU
}

void phase2Display() {
  menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 2:");

    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
    // simulate phase 1 ending
    delay(3000);
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("Complete!     ");
    delay(3000);
  menu.show();
}

void phase3Callback(uint16_t phas3) {
    // do something with the index
    // if selected "yes" do smth
    
    if (phas3 == 1) { // if 2nd index string is selected (Yes)
      phase3Display(); // hide menu and display new message of current active test phase
      phase3_int = 1;

    } else if (phas3 == 0) { // if 1rst index string is selected (No)
      //menu.show(); // Keep Menu active: don't start phase test
      phase3_int = 0;
    }
    Serial.println(phase3_int);
    //phas3 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 3 SUBMENU
}

void phase3Display() {
  menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 3:");

    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
    // simulate phase 1 ending
    delay(3000);
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("Complete!     ");
    delay(3000);
  menu.show();
}

// Sensor Read
void sensorRead(){
  //set ideal force
  //Events set to occur every 10 ms
  if ((millis()-savedTime1)%10==0){
    //sanity check
    if (idealForce<0){
      idealForce = 0;
    }
    Serial.print(idealForce);
    Serial.print(",");

//Sensor Read
    double xn = analogRead(SENSORREADPIN);
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
  }
}

//MOTOR CONTROL
void motorControl (int idealForce){
  //Motor control
    if ((millis()-savedTime1)%10==0){
      //Read analog RPM values from Motor Controller
      //Serial.print("Digital RPM Value = ");
      int rpmRead = analogRead(RPMREADPIN);
      // Serial.print("RPM Read Value: ");
      // Serial.println(rpmRead);
    
      //Read analog current values from Motor Controller
      //Serial.print("Digital Current Value = ");
      int currentRead = analogRead(CURRENTREADPIN);
      //Serial.print("Current Read Value: ");
      //Serial.println(currentRead);

      //only run of RPM is within bounds
      if ((rpmRead <= rpmLimit) && (rpmRead >=-rpmLimit) && (safety ==1)) {
        analogWrite(MOTORINPUTPIN,int(motorCurrent));
        Serial.println(int(motorCurrent));   
      }
      else {
        analogWrite(MOTORINPUTPIN,0);
        Serial.println("SAFETY");  
        safety = 0; //Safety Latch
      }
      //if sensor feels zero when supplied current, stop current from growing. Used to prevent the addition of force when finger isnt on properly.
      if ((sensorOutput<=500) && (currentRead>=currentLimit)) { 
        safety = 0; //Safety Latch
      }
    
      //get Error
      previousError = currentError; //saves previous error

      currentError = (idealForce - sensorOutput)/4095*255; 
    
      if (idealForce == 0){ //if statement used to prevent inconsistent controls if idealForce set to 0 for sometime due to the integral controller adding up wrong error. Force sensor does not stay at 0 when Idealforce =0.
        cumulativeError = cumulativeError + 0;
      }
      else{
        cumulativeError = cumulativeError + currentError; //get total error
      }
      derivativeError = (currentError - previousError); //difference between current and past error
      
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

void shuffleArray(double arr[], int n) {
  for (int i = n - 1; i > 0; i--) {
    // Pick a random index from 0 to i
    int j = random(0, i + 1);

    // Swap arr[i] with the element at random index
    double temp = arr[i];
    arr[i] = arr[j];
    arr[j] = temp;
  }
}