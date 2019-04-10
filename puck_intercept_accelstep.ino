#include <AccelStepper.h>
AccelStepper stepper(1,4,3); 
String inputString = "";         //a String to hold incoming data
String stkrCoord = "";
String pckCoord = "";

String printString = ""; //string to print back to serial terminal

int goalPos = 0; //middle position on air hockey table (step-wise)
bool stringComplete = false;  //whether the string is complete
bool retRead        = false;  //whether '\r' character has been read
bool calibration    = false;  //whether '?' has been read; true if ? exists

int currPos = 1050; //current position of the striker
int currDir = 0; //used to keep track of current direction, doesn't actually change direction of stepper motors

int stepSpeed = 400;



void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // Sets the two pins as Outputs
  stepper.setMaxSpeed(2000.0);
  stepper.setAcceleration(20000.0);
  
}

////////////
//  MAIN  //
////////////

void loop() {
  //new code starts here
  /*
 //check that striker is where it should be
  long stkrPosA = stepper.currentPosition(); //get where arduino thinks striker is
  //Serial.print("ArduinoPos: ");
  //Serial.println(stkrPosA);
  long stkrPosC = long(pckCoord.toInt());  //get where camera thinks striker is
  //Serial.print("CamPos: ");
  //Serial.println(stkrPosC);
  //make tolerances from this
  int lowTol = stkrPosC - 40; //lower tolerance
  int uprTol = stkrPosC + 40; //upper tolerance     
  //the following if statement makes things slow
  //if ((stkrPosA < lowTol) || (stkrPosA > uprTol)) //check that we're within tolerances
    //stepper.setCurrentPosition(long(pckCoord.toInt()));
*/
    //new code ends here
  
  // print the string when a newline arrives:
  if (stringComplete) {
    goalPos = stkrCoord.toInt();
    //currPos = pckCoord.toInt();

    //reset strings and flags
    inputString = "";
    stkrCoord = "";
    pckCoord = "";
    stringComplete = false;
    retRead = false;
  }

  //move to a position
   stepper.moveTo(goalPos);
   stepper.run();
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
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }

    //if the incoming character is a carriage return, set a flag so the incoming character
    //is appended to the correct substring (either stkrCoord or pckCoord)
    if (inChar == '\r'){
      retRead = true; //return char is read
    }
    
    // add it to the inputString:
    inputString += inChar;
    if(retRead){
      stkrCoord += inChar; //stkrCoord = where to move to
    } else {
      pckCoord += inChar; //pckCoord = striker's current position
    }
  }
}
