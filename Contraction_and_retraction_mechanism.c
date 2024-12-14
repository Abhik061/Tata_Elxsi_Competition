#include <Servo.h>
const int trigPin = 9;
const int echoPin = 10;
const int detectionThreshold = 20;
Servo myServo;
const int servoPin = 6;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(90); // Neutral position
}

void loop() {
  long duration = measureDistance();
  float distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  if (distance > 0 && distance <= detectionThreshold) {
    myServo.write(0); 
  } else {
    myServo.write(180); // Rotate to 180 degrees
  }
  delay(500);
}

long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH);
}
