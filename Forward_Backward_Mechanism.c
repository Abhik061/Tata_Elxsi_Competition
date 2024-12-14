const int enA = 9;   
const int in1 = 8;   
const int in2 = 7;   

const int enB = 10;  
const int in3 = 6;   
const int in4 = 5;   

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  driveMotorA(255, true); 
  driveMotorB(255, true); 
  delay(2000);            

  driveMotorA(200, false); 
  driveMotorB(200, false); 
  delay(2000);             
  stopMotorA();
  stopMotorB();
  delay(2000);             
}

void driveMotorA(int speed, bool forward) {
  if (forward) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, speed); 
}

void driveMotorB(int speed, bool forward) {
  if (forward) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB, speed); 
}

void stopMotorA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0); 
}

void stopMotorB() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0); 
}
