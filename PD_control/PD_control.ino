#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define SAMPLING_INTERVAL 20

#define WARMUP_SECONDS 6

#define ALPHA  0.5

#define NOISE_COEFF 0.5
#define LOOKBACK_AMOUNT 8

#define SENSOR_OFFSET 15
#define A 103  // distance measure at 10cm
#define B 360  // distance measure at 40cm
#define BALL_RADIUS 20

#define _DUTY_MIN 553 // servo full clockwise position (0 degree), actual is 10 degrees
#define _DUTY_MIN_DEG 10.0
#define _DUTY_NEU 1460 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree), actual is 172 degrees
#define _DUTY_MAX_DEG 172.0
#define _SERVO_SPEED 60 // servo speed limit (unit: degree/second)
#define _SERVO_INTERVAL 20
Servo myservo;
float duty_chg_per_interval, duty_target, duty_curr; // maximum duty difference per interval


#define _DUTY_MIN_BOUND 1850  // lowest servo angle
#define _DUTY_MAX_BOUND 950  // highest servo angle

#define _DIST_TARGET_MIN 270 // target distance
#define _DIST_TARGET_MAX 275
#define _KP 0.021
#define _KD 0.06
float error_curr, error_last = 0;


float x_out, x_current = 0, x_last = 0, v_current = 0, v_last = 0, a_current = 0, a_last = 0, x_stable, v_stable;
float acc_lookback[LOOKBACK_AMOUNT];
float vel_lookback[LOOKBACK_AMOUNT];
int lookback_index = 0;

float a_sum = 0, a_mean;

int sampling_count = 0;
unsigned long last_sampling_time = millis(), start_time, last_servo_time = millis();

float ema_agg = 0.0, dist_ema;

float deg2duty(float rail_deg){
  if(abs(rail_deg) < 0.1) return _DUTY_NEU;
  const float duty_per_deg = (_DUTY_MAX - _DUTY_MIN)/(_DUTY_MAX_DEG-_DUTY_MIN_DEG);
  const float servo_deg = 0.8607614114944276 + rail_deg * 3.1058577946956762;
  //Serial.println(servo_deg);
  return _DUTY_NEU + duty_per_deg * servo_deg;
}


float calculate_error(float curr_dist){
  if((_DIST_TARGET_MIN < curr_dist) && (_DIST_TARGET_MAX > curr_dist)){
    //Serial.print("in_rage");
    return 0;
  }
  else if(_DIST_TARGET_MIN > curr_dist){
    //Serial.print("undershoot");
    return _DIST_TARGET_MIN - curr_dist;
  }
  else if(_DIST_TARGET_MAX < curr_dist){
    //Serial.print("overshoot");
    return _DIST_TARGET_MAX - curr_dist;
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;// + SENSOR_OFFSET + BALL_RADIUS;
}

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  
// initialize serial port
  Serial.begin(57600);

  start_time = millis();

  for(int i = 0; i < LOOKBACK_AMOUNT; i++){
    acc_lookback[i] = NAN;
    vel_lookback[i] = NAN;
  }

  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _SERVO_INTERVAL / 1000;
}

int is_interpolate = 0;

void loop() {
  unsigned long time_curr = millis();
  if(last_servo_time + _SERVO_INTERVAL > time_curr){
    last_servo_time = time_curr;
      if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
      else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
      }
      myservo.writeMicroseconds(duty_curr);
  }
  if(last_sampling_time + SAMPLING_INTERVAL > time_curr) return;
  last_sampling_time = time_curr;
  float raw_dist = ir_distance();
  x_current = exp((log(raw_dist) - 1.9576)/0.7051) + 52.0695 + SENSOR_OFFSET + BALL_RADIUS;

  float v_current = (x_current - x_last)/SAMPLING_INTERVAL;
  float a_current = (v_current - v_last)/SAMPLING_INTERVAL;

  if(time_curr < start_time + 1000 * WARMUP_SECONDS){
    sampling_count++;
    a_sum = a_sum + abs(a_current);
    a_mean = a_sum / sampling_count;
    x_last = x_current;
    v_last = v_current;
    a_last = a_current;
    return;
  }
  else{
    // inertial interpolation
    if(abs(a_current) > a_mean * NOISE_COEFF){
      float acc_lookback_mean = 0, vel_lookback_mean = 0;
      int lookback_count = 0;
      for(int i = 0; i < LOOKBACK_AMOUNT; i++){
          if (isnan(acc_lookback[i])) continue;
  
          acc_lookback_mean = acc_lookback_mean + acc_lookback[i];
          vel_lookback_mean = vel_lookback_mean + vel_lookback[i];
          lookback_count++;
      }
      if(is_interpolate == 0) v_stable = (vel_lookback_mean/lookback_count) * SAMPLING_INTERVAL + acc_lookback_mean/lookback_count * SAMPLING_INTERVAL;
      else v_stable = v_stable + acc_lookback_mean/lookback_count * SAMPLING_INTERVAL;
      x_stable = x_stable + v_stable;
      x_out = x_stable;
      is_interpolate = 100;
    }
    else{
      if(is_interpolate == 100){
        for(int i = 0; i < LOOKBACK_AMOUNT; i++){
          acc_lookback[i] = NAN;
          vel_lookback[i] = NAN;
        }
      }
      x_out = x_current;
      x_stable = x_out;
      acc_lookback[lookback_index % LOOKBACK_AMOUNT] = a_current;
      vel_lookback[lookback_index % LOOKBACK_AMOUNT] = v_current;
      lookback_index++;
      is_interpolate = 0;
    }
  }

  x_out = isnan(x_out)? x_current: x_out;
  if(x_out <= 150 || x_out >= 400){
    x_out = x_current;
  }
  dist_ema = ALPHA * x_out + (1.0 - ALPHA) * ema_agg;
  ema_agg = dist_ema;

  //error_curr = _DIST_TARGET - dist_ema;
  error_curr = calculate_error(dist_ema);
  float pterm = _KP * error_curr;
  float dterm = _KD * (error_curr - error_last);
  duty_target = min(max(deg2duty(pterm + dterm), _DUTY_MAX_BOUND), _DUTY_MIN_BOUND);
//  Serial.print(" raw_dist:");
//  Serial.print(raw_dist);
//  Serial.print(" x_current:");
//  Serial.print(x_current);
//  Serial.print(" dist_ema:");
//  Serial.print(dist_ema, 4);
//  Serial.print(" duty_target:");
//  Serial.print(duty_target);
  Serial.print(" error_curr:");
  Serial.print(error_curr);
//  Serial.print(" angle:");

//  Serial.print(" duty_per_deg");
//  Serial.print((_DUTY_MAX - _DUTY_MIN)/(_DUTY_MAX_DEG-_DUTY_MIN_DEG));
  Serial.print(" Low:200 High:350 dist:");
  Serial.print(dist_ema);
  Serial.print(" pterm:"); 
  Serial.print(pterm);
  Serial.print(" duty_target:");
  Serial.print(duty_target);
  Serial.print(" duty_curr:");
  Serial.println(duty_curr);


  x_last = x_current;
  v_last = v_current;
  a_last = a_current;
  error_last = error_curr;
}
