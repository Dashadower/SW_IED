#include <Servo.h>
#define PIN_SERVO 10
#define DELAY 536
Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  delay(1000);
}

void loop() {
    // add code here.
    myservo.write(90);
    delay(DELAY);
    myservo.write(180);
    delay(DELAY);
    myservo.write(90);
    delay(DELAY);
    myservo.write(0);
    delay(DELAY);
}
