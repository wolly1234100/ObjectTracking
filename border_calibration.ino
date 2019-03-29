
String inputString = "";         //a String to hold incoming data
String stkrCoord = "";
String pckCoord = "";

String printString = ""; //string to print back to serial terminal

int goalPos = 1050; //middle position on air hockey table (step-wise)
bool stringComplete = false;  //whether the string is complete
bool retRead        = false;  //whether '\r' character has been read
bool calibration    = false;  //whether '?' has been read; true if ? exists

//define pin numbers
const int stepPin = 4; //cycles high low to send square wave and step once
const int dirPin = 3; //direction in which to move

int currPos = 1050; //current position of the striker
int currDir = 0; //used to keep track of current direction, doesn't actually change direction of stepper motors



void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  
  //for calibration, set initial direction to bottom (towards step 0 in y direction)
  currDir = 1; //var doesnt actually change directions
  digitalWrite(dirPin, LOW); //tell motors to go left
}

////////////
//  MAIN  //
////////////

void loop() {
  
  
  //take one step towards goal position
  if(!calibration){
    calibStep(); //calibration mode step funciton
  } else {
    // step to the center
    stepTo();
    Serial.println("current position, goal position: ");
    Serial.print(currPos, DEC);
    Serial.print(goalPos, DEC);
  }

  // print the string when a newline arrives:
  if (stringComplete) {
    //goalPos = stkrCoord.toInt();
    //currPos = pckCoord.toInt();

    //print results to serial console
    //printString = "Goal: " + stkrCoord + "Current Pos: " + pckCoord;
    //Serial.println(printString);

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
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }

    //if the incoming character is a carriage return, set a flag so the incoming character
    //is appended to the correct substring (either stkrCoord or pckCoord)
    if (inChar == '\r'){
      retRead = true;
    }

     //if the incoming character is '!', //UPDATE COMMENT
    if (inChar == '!'){
      delay(500);
      digitalWrite(dirPin, HIGH); //move in opp dir
      currDir = 1;     
      
    }

    //if the incoming character is '?',//UPDATE COMMENT
    if (inChar == '?'){
      calibration = true;
      goalPos = 1050;
      currPos = 0; //step0y in open cv coords
    }
    
    // add it to the inputString:
    inputString += inChar;
    if(retRead){
      stkrCoord += inChar;
    } else {
      pckCoord += inChar;
    }
    
    
  }
}

//move striker one step towards desired position
void stepTo(){
  int stepsToMove = goalPos - currPos; //vars declared at top
  if(stepsToMove > 0){
    digitalWrite(dirPin,LOW); //left
    currDir = 0;
  } else { //steps to move less than 0
    digitalWrite(dirPin,HIGH); //right
    currDir = 1;
  }
  //if there are steps to move, move one step
  if(abs(stepsToMove) > 0){ //take step
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500);
    if(currDir == 0){ //left increases steps
      currPos++;
    } else { //go right decrease steps
      currPos--;
    }
  }
}

void calibStep(){ //one step
  digitalWrite(stepPin,HIGH); 
  delayMicroseconds(500); 
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(500);
}
