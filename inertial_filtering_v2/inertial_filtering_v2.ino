// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define SAMPLING_INTERVAL 50

#define WARMUP_SECONDS 10
#define NOISE_COEFF 0.5
#define LOOKBACK_AMOUNT 8
int a, b; // unit: mm

float x_out, x_current = 0, x_last = 0, v_current = 0, v_last = 0, a_current = 0, a_last = 0, x_stable, v_stable;
float acc_lookback[LOOKBACK_AMOUNT];
float vel_lookback[LOOKBACK_AMOUNT];
int lookback_index = 0;

double a_sum = 0, a_mean;

int sampling_count = 0;
unsigned long last_sampling_time, start_time;


float ema_agg = 0.0, dist_ema;
#define ALPHA  0.35

int sign(float val){
  if (val > 0)return 1;
  else if (val < 0) return -1;
  else return 0;  
}

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 103;
  b = 443;

  start_time = millis();

  for(int i = 0; i < LOOKBACK_AMOUNT; i++){
    acc_lookback[i] = NAN;
    vel_lookback[i] = NAN;
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

int is_interpolate = 0;
void loop() {
  unsigned long time_curr = millis();
  if(last_sampling_time + SAMPLING_INTERVAL > time_curr) return;
  last_sampling_time = time_curr;
  float raw_dist = ir_distance();
  x_current = 100 + 300.0 / (b - a) * (raw_dist - a);

  float v_current = (x_current - x_last)/SAMPLING_INTERVAL;
  float a_current = (v_current - v_last)/SAMPLING_INTERVAL;

  if(time_curr < start_time + 1000 * WARMUP_SECONDS){
    sampling_count++;
    a_sum = a_sum + abs(a_current);
    a_mean = a_sum / sampling_count;

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

  dist_ema = ALPHA * x_out + (1.0 - ALPHA) * ema_agg;
  ema_agg = dist_ema;

  Serial.print("x_current:");
  Serial.print(x_current);
  Serial.print(" x_out:");
  Serial.print(x_out, 4);
  //Serial.print(" x_ema:");
  //Serial.print(dist_ema, 4);
  Serial.print(" is_interpolate:");
  Serial.print(is_interpolate);
  Serial.print(" v_t:");
  Serial.print(v_current, 4);
  Serial.print(" a_t:");
  Serial.println(a_current, 4);
  x_last = x_current;
  v_last = v_current;
  a_last = a_current;
}
