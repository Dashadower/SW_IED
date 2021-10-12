#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 536 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)
#define _DUTY_NEU (_DUTY_MIN + _DUTY_MAX) / 2 // servo neutral position (90 degree)

#define ALPHA  0.5  // ema alpha value


// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float ema_agg = 0.0;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until n ext sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  dist_ema = ALPHA * dist_raw + (1.0 - ALPHA) * ema_agg;  // calculate EMA value
  ema_agg = dist_ema;

// output the read value to the serial port
  Serial.print("Min:");
  Serial.print(_DIST_MIN);
  Serial.print(",Low:180,raw:");
  Serial.print(dist_raw);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.print(",High:220,Max:");
  Serial.println(_DIST_MAX);

// adjust servo position according to the USS read value

  // add your code here!

  if(dist_raw <= _DIST_MIN) {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if(dist_raw > _DIST_MIN && dist_raw < _DIST_MAX){
    myservo.writeMicroseconds(((_DUTY_MAX - _DUTY_MIN) / 180 * (dist_raw - 180)) + _DUTY_MIN);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  //if(reading < dist_min || reading > dist_max) {reading = 0.0;} // return 0 when out of range.
  if(reading >= dist_max) {
    reading = dist_max;
    digitalWrite(PIN_LED, HIGH);
  }
  else if(reading <= dist_min) {
    reading = dist_min;
    digitalWrite(PIN_LED, HIGH);
  }
  else {
    dist_prev = reading;
    digitalWrite(PIN_LED, LOW);
  }
  return reading;
}
