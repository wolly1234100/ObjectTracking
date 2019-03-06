const int dir = 3; //pins to go to motor driver
const int stepp = 4; 

int microseconds_delay;

char input;
boolean newData = false;
 
void setup() {

  //set pins as outputs
  pinMode(stepp,OUTPUT); 
  pinMode(dir,OUTPUT);

  //set direction pin high to choose direction
  digitalWrite(dir,HIGH);

  //begin serial connection
  Serial.begin(9600);

  //set initial speed
  microseconds_delay = 1000;
  
}
void loop() {

    //get input data and update variables
    recvOneChar();
    showNewData();

    //continually send steps to step pin on arduino
    digitalWrite(stepp,HIGH); 
    delayMicroseconds(microseconds_delay); 
    digitalWrite(stepp,LOW); 
    delayMicroseconds(microseconds_delay);
}

//take in one char of data at a time
void recvOneChar() {
 if (Serial.available() > 0) {
 input = Serial.read();
 newData = true;
 }
}

//for each received char print it back to serial and set step period
void showNewData() {
 if (newData == true) {

 Serial.print(input);
 if(input == 49){
  microseconds_delay = 500;
 }
 if(input == 50){
  microseconds_delay = 1000;
 }
 if(input == 51){
  microseconds_delay = 1500;
 }
 if(input == 52){
  microseconds_delay = 2000;
 }
 newData = false;
 }
}
