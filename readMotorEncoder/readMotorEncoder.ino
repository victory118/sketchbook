
// encoder variables
//const int chA = 5;
//const int chB = 6;
const int clkPinA = 2;//2
const int dirPinA = 4;//4
volatile int encoderACount = 0;
volatile boolean changeFlag = false;

const int motorClkPin = 3;//3
const int motorDirPin = 10;//1
volatile int motorEncoderCount = 0;
volatile boolean motorChangeFlag = false;

const int dirPin1 = 6;
const int speedPin1 = 9;

double posM = 0;

int analogPin = 3;
int val = 0;

int flag1 = 1;
int flag2 = 1;

void setup() {
  Serial.begin(9600);
  pinMode(clkPinA, INPUT);  
  pinMode(dirPinA, INPUT);
  
  pinMode(motorClkPin, INPUT);
  pinMode(motorDirPin, INPUT);
  
  //pinMode(chA, INPUT);
  //pinMode(chB, INPUT);
  attachInterrupt(0, encoderIntA, RISING);
  attachInterrupt(1, motorEncoderInt, RISING);
  
  pinMode(dirPin1,OUTPUT);
  pinMode(speedPin1,OUTPUT);
}

void loop() {
  
  delay(1000);
  Serial.println(motorEncoderCount);
  
  while (flag1 == 1) {
    digitalWrite(dirPin1,LOW);
    analogWrite(speedPin1,65);
    //Serial.println(encoderACount);
    Serial.println(motorEncoderCount);
    delay(100);
    if (motorEncoderCount < -1125) {
      flag1 = 0;
      analogWrite(speedPin1,0);
      break;
    }
  }
  delay(1000);
  Serial.println(motorEncoderCount);
  
  while (flag2 == 1) {
    digitalWrite(dirPin1,HIGH);
    analogWrite(speedPin1,65);
    Serial.println(motorEncoderCount);
    delay(100);
    if (motorEncoderCount > 0) {
      flag2 = 0;
      analogWrite(speedPin1,0);
      Serial.println(motorEncoderCount);
      break;
    }
  }
  delay(1000);
  Serial.println(motorEncoderCount);
  
  // 1124.5 counts is 8.8 inches on the track
  /*if (motorEncoderCount < 1125) {
    val = analogRead(analogPin); // reads from 0 to 1023
    digitalWrite(dirPin1,HIGH);
    analogWrite(speedPin1,65); // writes 0 to 255
  }
  else {
    analogWrite(speedPin1,0); // writes 0 to 255
  }
  posM = motorEncoderCount/5031.0;
  Serial.println(motorEncoderCount);
  Serial.println(posM);
  delay(100);*/
  
  /*if (changeFlag) {
    changeFlag = false;
    Serial.print("Encoder count = ");
    Serial.println(encoderACount);
  }*/
  
  /*if (motorChangeFlag) {
    motorChangeFlag = false;
    Serial.print("Encoder count = ");
    Serial.println(motorEncoderCount);
  }*/
  
//  Serial.print(digitalRead(chA));
//  Serial.println(digitalRead(chB));
  //Serial.print("Encoder count = ");
  //Serial.println(motorEncoderCount);
  //delay(1);
}

void encoderIntA() {
  if (digitalRead(dirPinA) == HIGH)
    encoderACount++;
  else
    encoderACount--;
  changeFlag = true;
}

void motorEncoderInt() {
  if (digitalRead(motorDirPin) == HIGH)
    motorEncoderCount++;
  else
    motorEncoderCount--;
  motorChangeFlag = true;
}




