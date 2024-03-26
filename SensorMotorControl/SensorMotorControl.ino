//LCD Menu definitions
#include <ItemInput.h>
#include <ItemList.h>
#include <ItemSubMenu.h>
#include <LcdMenu.h>  // Always comes after every item type import
#include <SPI.h>
#include <SdFat.h>
#include <ESP32Time.h>

const uint8_t BASE_NAME_SIZE = 26; // Length of "ExperimentXX"
uint8_t experimentNumber = 1; // Initialize experiment number

/* Pins */
const int microSD = 5;
const int emergencyStop = 25; // Pin for emergency stop button

ESP32Time rtc(3600);  // offset in seconds GMT+1
#define FILE_BASE_NAME "Data"
#define error(msg) sd.errorHalt(F(msg))

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


bool headerprinted = false;
bool fileCreated = false;

// RANGE SubMenu
// Declare the array
extern String range[];
// Initialize the array
String range[] = { "1", "2", "3", "4", "5" };

// REPETITIONS SubMenu
// Declare the array
extern String repetitions[];
// Initialize the array
String repetitions[] = { "3", "4", "5" };

// DURATION SubMenu
// Declare the array
extern String duration[];
// Initialize the array
String duration[] = { "3", "4", "5", "6", "7", "8" };

// PHASE 1 TEST SubMenu
extern String phase1[];
// Initialize the array
String phase1[] = { "No", "Yes" };

// PHASE 2 TEST SubMenu
extern String phase2[];
// Initialize the array
String phase2[] = { "No", "Yes" };

// PHASE 3 TEST SubMenu
extern String phase3[];
// Initialize the array
String phase3[] = { "No", "Yes" };

// RESET SAFETY SubMenu
extern String phase3[];
// Initialize the array
String safety_array[] = { "Yes" };

// Declare the call back function
void rangeCallback(uint16_t ran);
void repetitionsCallback(uint16_t reps);
void durationCallback(uint16_t dur);
void phase1Callback(uint16_t phas1);
void phase2Callback(uint16_t phas2);
void phase3Callback(uint16_t phas3);
void safetyCallback(uint16_t safe);


//Values received from UI, saved to use for the program
int range_int = 0;
int repetitions_int = 0;
int duration_int = 0;

int safety_reset = 0;

//Prev current counters, used to show that a range was selected. However, the researcher cannot reselect the same range.
int prev_range = 0;

int phase1_Start = 0;                 //latch to start phase 1 ONCE
int phase1_InProgress = 0;            //show that phase 1 is currently running
unsigned long phase1_ShowTime = 0;    //Used for phase 1 diplay
unsigned long phase1_RunningTime = 0;    //Used for timing to send idealForces

int phase2_Start = 0;                 //latch to start phase 2 ONCE
int phase2_InProgress = 0;    
unsigned long phase2_ShowTime = 0;    //Used for phase 2 diplay
unsigned long phase2_RunningTime = 0;    //Used for timing to record forces

int phase3_Start = 0;                 //latch to start phase 3 ONCE
int phase3_InProgress = 0;    
unsigned long phase3_ShowTime = 0;    //Used for phase 3 diplay
unsigned long phase3_RunningTime = 0;    //Used for timing to record forces

//Menu Items
extern MenuItem* rangeMenu[];
extern MenuItem* repetitionsMenu[];
extern MenuItem* durationMenu[];
extern MenuItem* phase1Menu[];
extern MenuItem* phase2Menu[];
extern MenuItem* phase3Menu[];
extern MenuItem* safetyMenu[];


MAIN_MENU(  // DON'T FORGET THE COMMA !!!!!!!
  ITEM_SUBMENU("Range (N)", rangeMenu),
  ITEM_SUBMENU("Test Repetitions", repetitionsMenu),
  ITEM_SUBMENU("Duration", durationMenu),
  ITEM_SUBMENU("Phase 1", phase1Menu),
  ITEM_SUBMENU("Phase 2", phase2Menu),
  ITEM_SUBMENU("Phase 3", phase3Menu),
  ITEM_SUBMENU("Reset Safety", safetyMenu));


/**
 * Create submenu and precise its parent
 */
SUB_MENU(rangeMenu, mainMenu,
         ITEM_STRING_LIST("Range (N)", range, 5, rangeCallback));

SUB_MENU(repetitionsMenu, mainMenu,
         ITEM_STRING_LIST("# of Reps.", repetitions, 3, repetitionsCallback));

SUB_MENU(durationMenu, mainMenu,
         ITEM_STRING_LIST("Duration (s)", duration, 6, durationCallback));

SUB_MENU(phase1Menu, mainMenu,
         ITEM_STRING_LIST("Start test?", phase1, 2, phase1Callback));

SUB_MENU(phase2Menu, mainMenu,
         ITEM_STRING_LIST("Start test?", phase2, 2, phase2Callback));

SUB_MENU(phase3Menu, mainMenu,
         ITEM_STRING_LIST("Start test?", phase3, 2, phase3Callback));

SUB_MENU(safetyMenu, mainMenu,
         ITEM_STRING_LIST("Reset Safety?", safety_array, 1, safetyCallback));

LcdMenu menu(LCD_ROWS, LCD_COLS);

// JACKY PART!!!!!!
//motorControl Define DAC and ADC pins
#define MOTORINPUTPIN 33  //used to be 26
#define CURRENTREADPIN 34
#define SENSORREADPIN 35
#define RPMREADPIN 32
#define SLIDERPIN 15 //DOUBLE CHECK PIN FOR SLIDER!!!
#define MicroSD 5
#define emergencyStop 25

//motorControl past values for filtering
double yn1 = 0;
double yn2 = 0;
double yn3 = 0;
double xn1 = 0;
double xn2 = 0;
double xn3 = 0;
double sensorOutput = 0;  //force sensor reading
double sliderOutput = 0; //slider reading

//motorControl filter coefficients
double b[5] = { 0.213398512073359, 0.640195536174559, 0.640195536307786, 0.213398512030061 };
double a[5] = { 1.000000000000000, -2.918393191953221, 2.842867652821765, -0.924303742058885 };

//motorControl software limit
double rpmLimit = 2000;      //RPM safety ADC value
double currentLimit = 3000;  //Current safety ADC value
//motorControl error PID terms
double currentError = 0;     //for P term
double cumulativeError = 0;  //for I term
double previousError = 0;    //for D term
double derivativeError = 0;  //for D term

//motorControl error constants CHANGE THEM HERE; OR CHANGE THE ACTUAL LOGIC DOWN IN MOTOR CONTROLLER ERROR TERMS (Section: get Error)
double Kp = 0.010;
double Ki = 0.000001;
double Kd = 0.000001;
//motorControl items
double motorCurrent = 0;  //DAC value
int safety = 0;           //safety latch for when RPM or current (special case, see motorcontrol section) goes over limit. Need to reset esp32 to reset

double idealForce = 0;           //set force value
unsigned long savedTime1 = 0;    //Used for specific timing for sampling freq
unsigned long savedTime2 = 0;   //used for data capturef


int lengthForceArray = 0;        //get the length of the array, a constant
int idealForceArrayCounter = 0;  //used to count what place in idealForceArray we are in
int phase1_Delay = 2000;         //small delay for when phase 1 is selected and the motor acting on that force
int phase2_Delay = 2000;         //small delay for when phase 2 is selected and the sensor is recording the force
int phase3_Delay = 2000;         //small delay for when phase 2 is selected and the motor is recording the force


int idealForceArray[9];
int idealForceADC[9];

//Data Export Variables
SdFat sd;        //SD Card Variable
File dataFile;   //Data File Variable
bool experimentRunning = true; //controls the overall state of the experiment
bool initializeSD(); //initializes the SD card for communication. Returns true if initialization is sucessful, false otherwise
bool initializeDataFile(uint8_t phase); //opens a data file on the SD card for writing. It creates a new file named with the timestamp of the experiment start time. Returns true if the file iis sucessfully opened, false otherwise.
void checkEmergencyStopButton(); //checks if the emergency stop button is pressed. If pressed, it stops the experiment and logs a message indicating the experiment is over. 
#define FILE_BASE_NAME "Data"
#define error(msg) sd.errorHalt(F(msg))

double sensorOutput_Newtons;

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
  for (int i = 0; i < (sizeof(b) / sizeof(b[0])); i++) {
    b[i] = b[i] * 0.0001;
  }
    rtc.setTime(0, 0, 1, 24, 3, 2024, 0);  
  //Initialize SD card
    if (!initializeSD()) {
        Serial.println("SD Card initialization failed.");
        return;
      }
// // Open data file
//   if (!initializeDataFile()) {
//     Serial.println("Error opening data file.");
//     return;
//   }
  //save start time
  savedTime1 = millis();
}

void loop() {  //Loop Starts Here --------------------------------------------


  //if past range input != new range input
  if (range_int != prev_range) {

    if (range_int == 1) {  //look up tables depending on what was the input
      double idealForceArray[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
      lengthForceArray = sizeof(idealForceArray) / sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);  //randomizes the array.
      for (int i = 0; i < lengthForceArray; i++) {
        idealForceADC[i] = 29.9836 * pow(idealForceArray[i], 2) + 595.772 * idealForceArray[i] - 14.7438;  //quadratic line of best fit to change Force into ADC Value based on previous testing !! can always be changed.
      }
    } else if (range_int == 2) {  //look up tables depending on what was the input
      double idealForceArray[] = { 1, 1.5, 2, 0, 0, 0, 0, 0, 0 };
      lengthForceArray = sizeof(idealForceArray) / sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      for (int i = 0; i < lengthForceArray; i++) {
        idealForceADC[i] = 29.9836 * pow(idealForceArray[i], 2) + 595.772 * idealForceArray[i] - 14.7438;
      }
    } else if (range_int == 3) {  //look up tables depending on what was the input
      double idealForceArray[] = { 1, 1.5, 2, 2.5, 3, 0, 0, 0, 0 };
      lengthForceArray = sizeof(idealForceArray) / sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      for (int i = 0; i < lengthForceArray; i++) {
        idealForceADC[i] = 29.9836 * pow(idealForceArray[i], 2) + 595.772 * idealForceArray[i] - 14.7438;
      }
    } else if (range_int == 4) {  //look up tables depending on what was the input
      double idealForceArray[] = { 1, 1.5, 2, 2.5, 3, 3.5, 4, 0, 0 };
      lengthForceArray = sizeof(idealForceArray) / sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      for (int i = 0; i < lengthForceArray; i++) {
        idealForceADC[i] = 29.9836 * pow(idealForceArray[i], 2) + 595.772 * idealForceArray[i] - 14.7438;
      }
    } else if (range_int == 5) {  //look up tables depending on what was the input
      double idealForceArray[] = { 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5 };
      lengthForceArray = sizeof(idealForceArray) / sizeof(idealForceArray[0]);
      shuffleArray(idealForceArray, lengthForceArray);
      for (int i = 0; i < lengthForceArray; i++) {
        idealForceADC[i] = 29.9836 * pow(idealForceArray[i], 2) + 595.772 * idealForceArray[i] - 14.7438;
      }
    } else {
      Serial.print("Error");
    }
    prev_range = range_int;      //past input = new input
    idealForceArrayCounter = 0;  //counter to track what position we are in
  }


  //=phase 1=
  if ((phase1_Start == 1) && ((millis() - phase1_RunningTime) >= phase1_Delay) && (lengthForceArray > 0)) {                //gets ideal force on Phase 1, with a delay of phase1_Delay, and range has been selected
    if ((idealForceArrayCounter < lengthForceArray) && (idealForceADC[idealForceArrayCounter] > 0)) {              //if counter is less than total length of the look up table, and if the Force value is greater than 0, do this:
      idealForce = idealForceADC[idealForceArrayCounter];                                                          //set ideal force to value i in look up table
      idealForceArrayCounter++;                                                                                    //increment counter
    } else if ((idealForceADC[idealForceArrayCounter] <= 0) && (idealForceArrayCounter < lengthForceArray - 1)) {  //else if Force value is less than 0, and counter is less than length-1, skip it until we hit a positive Force value
      while ((idealForceADC[idealForceArrayCounter] <= 0) && (idealForceArrayCounter < lengthForceArray - 1)) {    //skipping part
        idealForceArrayCounter++;
      }
      idealForce = idealForceADC[idealForceArrayCounter];   //prints first non zero value
      if (idealForceArrayCounter < lengthForceArray - 1) {  //increment counter right after
        idealForceArrayCounter++;
      }
    } else {
      idealForce = 0;  //else Set force to 0, usually after the sequence is finished.
    }
    phase1_Start = 0;
    savedTime2 = millis();
  }
  //=phase 1 stop and complete screen=
  if ( ((millis() - phase1_RunningTime) >= (phase1_Delay + duration_int * 1000)) && (phase1_InProgress==1))  {  //motor will exert 1 force chosen from above code, and then will exert that for duration_int length, then stop.
    idealForce = 0;
    phase1_InProgress = 0;
    if (phase1_InProgress == 0){
      menu.lcd->setCursor(0, 1);
      menu.lcd->print("Complete!     ");
      phase1_ShowTime = millis();
      headerprinted = false;
      dataFile.close();
    }
  }
   if (phase1_InProgress == 1 && !fileCreated){
    // Create a new file for phase 1
        if (!initializeDataFile(1)) {
            Serial.println("Error opening data file for phase 1.");
            return;
        }
        // Write header information to the file
        dataFile.println("Timestamp (ms), Applied Force (N)");
        fileCreated = true; // Mark the file as created for phase 1
    }
  
  if (phase1_InProgress == 1 && ((millis() - phase1_RunningTime) >= phase1_Delay) ) {
    // if (headerprinted == false) {
    //   dataFile.println("Timestamp (ms), Applied Force (N)");
    //   headerprinted = true;
    // }
    if ((millis() - savedTime2) >= 100) {
      dataFile.print(rtc.getTime()); // Real-time timestamp
      dataFile.print(",");
      dataFile.println(sensorOutput_Newtons);
      //delay(1000);
      savedTime2 = millis();
    }
  }
  //=phase 1 complete stop and go back to menu=
  if ((millis() - phase1_ShowTime) >= (2000) && (phase1_ShowTime!=0)){ 
    menu.show();
    phase1_ShowTime =0;
    fileCreated = false;
  }




  //=phase 2=
  if (phase2_Start == 1 && !fileCreated){
    // Create a new file for phase 1
        if (!initializeDataFile(2)) {
            Serial.println("Error opening data file for phase 1.");
            return;
        }
        // Write header information to the file
      dataFile.println("Timestamp (ms), Patient Applied Force (N)");
        fileCreated = true; // Mark the file as created for phase 1
    }
  if ((phase2_Start == 1) && ((millis() - phase2_RunningTime) >= phase2_Delay)) {                //copied paste from phase1, may not be efficient
    //-> RECORD DATA HERE
    if ((millis() - savedTime2) >= 100) {
      dataFile.print(rtc.getTime()); // Real-time timestamp
      dataFile.print(",");
      dataFile.println(sensorOutput_Newtons);
      //delay(1000);
      savedTime2 = millis();
    }
  
    idealForce = 0;
  }
  //=phase 2 stop and complete screen =
  if ( ((millis() - phase2_RunningTime) >= (phase2_Delay + duration_int * 1000)) && (phase2_InProgress==1))  {  
    phase2_InProgress = 0;
    phase2_Start = 0;
    if (phase2_InProgress == 0){
      menu.lcd->setCursor(0, 1);
      menu.lcd->print("Complete!     ");
      phase2_ShowTime = millis();
      //->STOP DATA RECORD HERE
      headerprinted = false;
      fileCreated = false;
      dataFile.close();
    }
  }
  //=phase 2 complete stop and go back to menu=
  if ((millis() - phase2_ShowTime) >= (2000) && (phase2_ShowTime!=0)){ 
    menu.show();
    phase2_ShowTime =0;
    
  }


    //=phase 3=
  if (phase3_Start == 1 && !fileCreated){
    // Create a new file for phase 1
        if (!initializeDataFile(3)) {
            Serial.println("Error opening data file for phase 3.");
            return;
        }
        // Write header information to the file
      dataFile.println("Timestamp (ms), Slider Applied Force (N)");
        fileCreated = true; // Mark the file as created for phase 1
    }
  if ((phase3_Start == 1) && ((millis() - phase3_RunningTime) >= phase3_Delay)) {                //when phase 3 selected, get idealForce from Slider
    
    idealForce = sliderOutput;
    
    // if (headerprinted == false) {
    //   dataFile.println("Timestamp (ms), Patient Applied Force (N)");
    //   headerprinted = true;
    // }
    if ((millis() - savedTime2) >= 100) {
      dataFile.print(rtc.getTime()); // Real-time timestamp
      dataFile.print(",");
      dataFile.println(sensorOutput_Newtons); //should this be idealForce?
      savedTime2 = millis();
    }
  }
  //=phase 3 stop and complete screen =
  if ( ((millis() - phase3_RunningTime) >= (phase3_Delay + duration_int * 1000)) && (phase3_InProgress==1))  {  //motor will exert 1 force chosen from above code, and then will exert that for duration_int length, then stop.
    idealForce = 0;
    phase3_InProgress = 0;
    phase3_Start = 0;
    
    if (phase3_InProgress == 0){
      
      menu.lcd->setCursor(0, 1);
      menu.lcd->print("Complete!     ");
      phase3_ShowTime = millis();
      //->STOP DATA RECORD HERE
      headerprinted = false;
      fileCreated = false;
      dataFile.close();
    }
  }
  //=phase 3 complete stop and go back to menu=
  if ((millis() - phase3_ShowTime) >= (2000) && (phase3_ShowTime!=0)){ 
    menu.show();
    phase3_ShowTime =0;
  }
  
 

  //motor control always on right now, need to seperate sensor reading and motor control
  motorControl (idealForce);

  //UI STUFF ----------------------------------------
  char command;
  // reading buttons
  backcurr = digitalRead(backbut);  // keep in code
  downcurr = digitalRead(downbut);
  entercurr = digitalRead(enterbut);
  upcurr = digitalRead(upbut);
  //Serial.print(backcurr);
  if (backlast == HIGH && backcurr == LOW)
    command = 55;
  // save the last state
  backlast = backcurr;

  if (downlast == HIGH && downcurr == LOW)
    command = 50;
  // save the last state
  downlast = downcurr;

  if (enterlast == HIGH && entercurr == LOW)
    command = 53;
  // save the last state
  //enterlast = entercurr;

  if (uplast == HIGH && upcurr == LOW)
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
}

// CALLBACK FUNCTIONS
void rangeCallback(uint16_t ran) {
  // do something with the index

  range_int = (range[ran]).toInt();

}

void repetitionsCallback(uint16_t reps) {
  // do something with the index


  // convert string found at index 'reps' to int
  repetitions_int = (repetitions[reps]).toInt();

}

void durationCallback(uint16_t dur) {
  // do something with the index


  // convert string found at index 'dur' to int
  duration_int = (duration[dur]).toInt();


}

void phase1Callback(uint16_t phas1) {
  // do something with the index
  // if selected "yes" do smth

  if (phas1 == 1) {   // if 2nd index string is selected (Yes)
    
    phase1_RunningTime = millis();  //saves time
    phase1_Start = 1;
    phase1_InProgress=1;
    menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 1:");
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
  } else if (phas1 == 0) {  // if 1rst index string is selected (No)
    //menu.show // Keep Menu active: don't start phase test
    phase1_Start = 0;
  }
  // Serial.println(phase1_Start);
  //phas1 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 1 SUBMENU
}

void phase2Callback(uint16_t phas2) {
  if (phas2 == 1) {   // if 2nd index string is selected (Yes)
    phase2_RunningTime = millis();  //saves time
    phase2_Start = 1;
    phase2_InProgress=1;
    menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 2:");
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
  } else if (phas2 == 0) {  // if 1rst index string is selected (No)
    //menu.show // Keep Menu active: don't start phase test
    phase2_Start = 0;
  }
  // Serial.println(phase2_Start);
  //phas2 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 2 SUBMENU
}


void phase3Callback(uint16_t phas3) {
  if (phas3 == 1) {   // if 2nd index string is selected (Yes)
    phase3_RunningTime = millis();  //saves time
    phase3_Start = 1;
    phase3_InProgress=1;
    menu.hide();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Phase 3:");
    menu.lcd->setCursor(0, 1);
    menu.lcd->print("In Progress...");
  } else if (phas3 == 0) {  // if 1rst index string is selected (No)
    //menu.show // Keep Menu active: don't start phase test
    phase3_Start = 0;
  }
  // Serial.println(phase3_Start);
  //phas2 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 2 SUBMENU
}

void safetyCallback(uint16_t safe) {
  if (safe == 0) {   // if 2nd index string is selected (Yes)
    safety_reset = 1;
  }
  // Serial.println(phase3_Start);
  //phas2 = 0; FIGURE OUT HOW TO RESET BACK TO "NO" IN PHASE 2 SUBMENU
}

// MOTOR CONTROL
void motorControl(int idealForce) {
  //set ideal force
  //Events set to occur every 10 ms
  if ((millis() - savedTime1) % 10 == 0) {
    //sanity check
    if (idealForce < 0) {
      idealForce = 0;
    }
    Serial.print(idealForce);
    Serial.print(",");

    //Sensor Read
    double xn = analogRead(SENSORREADPIN);
    sliderOutput= analogRead(SLIDERPIN);
    //output value
    //4th order difference equation - based on matlab filter order
    //double yn = -a[1]*yn1 - a[2]*yn2 - a[3]*yn3 - a[4]*yn4 + b[0]*xn + b[1]*xn1 + b[2]*xn2 + b[3]*xn3 + b[4]*xn4;
    //3rd order difference equation - based on matlab filter order
    double yn = -a[1] * yn1 - a[2] * yn2 - a[3] * yn3 + b[0] * xn + b[1] * xn1 + b[2] * xn2 + b[3] * xn3;

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
    if (yn >= 0) {
      sensorOutput = yn;
    } else if (yn < 0) {
      sensorOutput = 0;
    }
    Serial.print(sensorOutput);
    sensorOutput_Newtons = sqrt((sensorOutput+2974.23062)/29.9836)-9.93496445;
    Serial.print(",");
    Serial.print(sensorOutput_Newtons);

    //Motor control

    //Read analog RPM values from Motor Controller
    int rpmRead = analogRead(RPMREADPIN);
    // Serial.print(",");
    // Serial.print(rpmRead);

    //Read analog current values from Motor Controller

    int currentRead = analogRead(CURRENTREADPIN);
    // Serial.print(",");
    // Serial.print(currentRead);

    Serial.print(",");
    Serial.print(safety);
    //only run if safety is off
    if (safety == 0) {
      analogWrite(MOTORINPUTPIN, int(motorCurrent));
      Serial.print(",");
      Serial.println((int)motorCurrent);
    } else {
      analogWrite(MOTORINPUTPIN, 0);
      Serial.print(",");
      Serial.println("SAFETY");
      
    }
    //if sensor feels zero when supplied current, stop current from growing. Used to prevent the addition of force when finger isnt on properly.
    if ( ((sensorOutput <= 1000) && (currentRead >= currentLimit)) || ((rpmRead >= rpmLimit) || (rpmRead <= -rpmLimit)) ) {
      safety = 1;  //Safety Latch
    }
    
    //reset safety via UI menu
    if (safety == 1){
      if (safety_reset ==1){
        safety = 0;
        safety_reset = 0;
        motorCurrent = 0;
      }
    }
    

    //get Error
    previousError = currentError;  //saves previous error

    currentError = (idealForce - sensorOutput) / 4095 * 255;

    if (idealForce == 0) {  //if statement used to prevent inconsistent controls if idealForce set to 0 for sometime due to the integral controller adding up wrong error. Force sensor does not stay at 0 when Idealforce =0.
      cumulativeError = 0;
      derivativeError = 0;
    }
    else if (abs(currentError)<0.25*idealForce){ //used to prevent integral windup during the transient response, which created steadystate error
      cumulativeError = cumulativeError + currentError;
      derivativeError = (currentError - previousError);
    }
     else {
      cumulativeError = 0;  //get total error
      derivativeError = (currentError - previousError);  //difference between current and past error
    }
    

    //apply error and limit motorCurrent to [0,255]
    if (((motorCurrent + Kp * currentError + Ki * cumulativeError + Kd * derivativeError) <= 255) && ((motorCurrent + Kp * currentError + Ki * cumulativeError + Kd * derivativeError) >= 0)) {
      motorCurrent = motorCurrent + Kp * currentError + Ki * cumulativeError + Kd * derivativeError;
    } else if ((motorCurrent + Kp * currentError + Ki * cumulativeError + Kd * derivativeError) > 255) {
      motorCurrent = 255;
    } else if ((motorCurrent + Kp * currentError + Ki * cumulativeError + Kd * derivativeError) < 0) {
      motorCurrent = 0;
    }
    // Serial.print(",");
    // Serial.print(currentError);
    // Serial.print(",");
    // Serial.print(cumulativeError);
    // Serial.print(",");
    // Serial.println(derivativeError);
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

/* initializeSD Function: initializes the SD card for communication. 
Returns true if initialization is sucessful, false otherwise */
bool initializeSD() {
  return sd.begin(microSD, SPI_HALF_SPEED);
}

/* initializeDataFile Function: opens a data file on the SD card for writing. 
It creates a new file named with the timestamp of the experiment start time. 
Returns true if the file iis sucessfully opened, false otherwise. */
// bool initializeDataFile() {
  

//   dataFile = sd.open("test02.csv", FILE_WRITE);
//   if (dataFile) {
//     // //writeHeaders();
//     dataFile.println("Time(ms), FSR Value");
//     Serial.println("Header written to file.");
//     //dataFile.close();
//     return true;
//   }
//   return false;
// }

/* initializeDataFile Function: opens a data file on the SD card for writing.
It creates a new file named with the timestamp of the experiment start time.
Returns true if the file is successfully opened, false otherwise. */

// bool initializeDataFile() {
//   //String filename = "test04.csv"; // Default filename
  
//   const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
//   char fileName[13] = FILE_BASE_NAME "00.csv";

//   // Find an unused file name.
//   if (BASE_NAME_SIZE > 6) {
//     error("FILE_BASE_NAME too long");
//   }
//   while (sd.exists(fileName)) {
//     if (fileName[BASE_NAME_SIZE + 1] != '9') {
//       fileName[BASE_NAME_SIZE + 1]++;
//     } else if (fileName[BASE_NAME_SIZE] != '9') {
//       fileName[BASE_NAME_SIZE + 1] = '0';
//       fileName[BASE_NAME_SIZE]++;
//     } else {
//       error("Can't create file name");
//     }
//   }
//   if (!dataFile.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
//     error("file.open");
//   }
//   return false;
// }

bool initializeDataFile(uint8_t phase) {  
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[26]; // Adjusted size to accommodate the longer string

  // Construct file name with experiment number and phase
  sprintf(fileName, "Data%02d_Phase%d.csv", experimentNumber, phase);
if (!fileCreated){
  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!dataFile.open(fileName, O_WRONLY | O_CREAT)) {
    error("file.open");
    return false;
  }
}

  // Increment experiment number for the next file
  experimentNumber++;

  return true; // Return true if the file is successfully opened
}


/* checkEmergencyStopButton Function: checks if the emergency stop button is pressed. 
If pressed, it stops the experiment and logs a message indicating the experiment is over. */
void checkEmergencyStopButton() {
  if (digitalRead(emergencyStop) == LOW) {
    experimentRunning = false;
    Serial.println("Emergency stop button activated. Experiment is over and Data has been successfully saved");
  }
}
