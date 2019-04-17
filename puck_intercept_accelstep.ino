#include <AccelStepper.h>
AccelStepper stepper1(1,4,3); 
AccelStepper stepper2(1,6,5);

int fbMarginSteps = 100;

String inputString = ""; //a String to hold incoming data
String stepper1Coord = "";
String stepper2Coord = "";
String fbStepper1Coord = "";
String fbStepper2Coord = "";

int step1Coord = 0;
int step2Coord = 0;
int fbStep1Coord = 0;
int fbStep2Coord = 0;

bool stringComplete = false;  //whether the string is complete
bool retRead        = false;  //whether '\r' character has been read
bool tabRead        = false;  //whether '\t' character has been read
bool spaceRead      = false;  //whether ' ' character has been read

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  stepper1.setMaxSpeed(3000.0);
  stepper1.setAcceleration(20000.0);
  stepper2.setMaxSpeed(3000.0);
  stepper2.setAcceleration(20000.0);
  
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    //convert strings to integers
    step1Coord = stepper1Coord.toInt();
    step2Coord = stepper2Coord.toInt();
    fbStep1Coord = fbStepper1Coord.toInt();
    fbStep2Coord = fbStepper2Coord.toInt();

    //reset strings and flags
    inputString = "";
    stepper2Coord = "";
    stepper1Coord = "";
    fbStepper2Coord = "";
    fbStepper1Coord = "";
    stringComplete = false;
    retRead = false;
    tabRead = false;
    spaceRead = false;
  }

   //feeback loop
   
   if ((stepper1.currentPosition() < fbStep1Coord - fbMarginSteps) ||
       (stepper1.currentPosition() > fbStep1Coord + fbMarginSteps) ||
       (stepper2.currentPosition() < fbStep2Coord - fbMarginSteps) ||
       (stepper2.currentPosition() > fbStep2Coord + fbMarginSteps))
       {
          stepper1.setCurrentPosition(fbStep1Coord);
          stepper2.setCurrentPosition(fbStep2Coord);
       }
     

       

   //move to a position
   stepper1.moveTo(step1Coord);
   stepper1.run();
   stepper2.moveTo(step2Coord);
   stepper2.run();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

     //if the incoming character is a carriage return, set a flag so the incoming character
    if (inChar == '\r'){
      retRead = true; 
    }

    //if the incoming character is a carriage return, set a flag so the incoming character
    if (inChar == '\t'){
      tabRead = true; 
    }

    //if the incoming character is a carriage return, set a flag so the incoming character
    if (inChar == ' '){
      spaceRead = true; 
    }
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    
    // add it to the inputString:
    inputString += inChar;

    
    if(retRead && !tabRead && !spaceRead){
      stepper2Coord += inChar;
    } else if(retRead && tabRead && !spaceRead) {
      fbStepper1Coord += inChar; 
    } else if(retRead && tabRead && spaceRead){
      fbStepper2Coord += inChar;
    } else {
      stepper1Coord += inChar;
    }
  }
}
