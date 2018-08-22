
// encoder variables
const int clkPinL = 2;
const int dirPinL = 8;
volatile int encoderCountL = 0;
volatile boolean changeFlagL = false;

const int clkPinR = 3;
const int dirPinR = 9;
volatile int encoderCountR = 0;
volatile boolean changeFlagR = false;

void setup() {
  Serial.begin(9600);
  pinMode(clkPinL, INPUT);  
  pinMode(dirPinL, INPUT);
  pinMode(clkPinR, INPUT);  
  pinMode(dirPinR, INPUT);
  attachInterrupt(0, encoderIntL, RISING);
  attachInterrupt(1, encoderIntR, RISING);
}

void loop() {
  if (changeFlagL) {
    changeFlagL = false;
    Serial.print("Left encoder count = ");
    Serial.println(encoderCountL);
  }
  
  if (changeFlagR) {
    changeFlagR = false;
    Serial.print("Right encoder count = ");
    Serial.println(encoderCountR);
  }
  
  delay(50);
}

void encoderIntL() {
  if (digitalRead(dirPinL) == HIGH)
    encoderCountL++;
  else
    encoderCountL--;
  changeFlagL = true;
}

void encoderIntR() {
  if (digitalRead(dirPinR) == HIGH)
    encoderCountR++;
  else
    encoderCountR--;
  changeFlagR = true;
}





