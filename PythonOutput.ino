
String inputString = "";         // a String to hold incoming data
String stkrCoord = "";
String pckCoord = "";

String printString = ""; //string to print back to serial terminal

int goalPos = 1050;
bool stringComplete = false;  // whether the string is complete
bool retRead = false; //whether '\r' character has been read

const int stepPin = 4; //define pins numbers
const int dirPin = 3; 

int currPos = 1050; //current position of the striker
int currDir = 0;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}

void loop() {
  //take one step towards goal position
  stepTo();

  // print the string when a newline arrives:
  if (stringComplete) {
    goalPos = stkrCoord.toInt();
    currPos = pckCoord.toInt();

    //print results to serial console
    printString = "Goal: " + stkrCoord + "Current Pos: " + pckCoord;
    Serial.println(printString);

    //reset strings and flags
    inputString = "";
    stkrCoord = "";
    pckCoord = "";
    stringComplete = false;
    retRead = false;
  }
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
    // add it to the inputString:
    inputString += inChar;
    if(retRead){
      stkrCoord += inChar;
    } else {
      pckCoord += inChar;
    }
    
    if (inChar == '\r'){
      retRead = true;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      
    }
  }
}

//move striker one step towards desired position
void stepTo(){
  int stepsToMove = goalPos - currPos;
  if(stepsToMove > 0){
    digitalWrite(dirPin,HIGH);
    currDir = 0;
  } else {
    digitalWrite(dirPin,LOW);
    currDir = 1;
  }
  //if there are steps to move, move one step
  if(abs(stepsToMove) > 0){
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500);
    //if(currDir == 0){
    //  currPos++;
    //} else {
    //  currPos--;
    //}
  }

  
}
