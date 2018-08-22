
// encoder variables
//const int chA = 5;
//const int chB = 6;
const int clkPinA = 2;
const int dirPinA = 8;
volatile int encoderACount = 0;
volatile boolean changeFlag = false;

void setup() {
  Serial.begin(9600);
  pinMode(clkPinA, INPUT);  
  pinMode(dirPinA, INPUT);
  //pinMode(chA, INPUT);
  //pinMode(chB, INPUT);
  attachInterrupt(0, encoderIntA, RISING);
}

void loop() {
  if (changeFlag) {
    changeFlag = false;
    Serial.print("Encoder count = ");
    Serial.println(encoderACount);
  }
  
//  Serial.print(digitalRead(chA));
//  Serial.println(digitalRead(chB));
  delay(50);
}

void encoderIntA() {
  if (digitalRead(dirPinA) == HIGH)
    encoderACount++;
  else
    encoderACount--;
  changeFlag = true;
}




