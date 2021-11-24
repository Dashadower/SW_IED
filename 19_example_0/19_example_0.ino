// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9

int a, b; // unit: mm
float opt_k0, dist_filtered;
void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 103;
  b = 443;
  opt_k0 = 100 + 300.0 / (b - a) * (ir_distance() - a);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float kalman_simplified(float raw_k1, float opt_k0){
  const float K = 0.25;
  return raw_k1 * K + (1 - K) * opt_k0;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_filtered = kalman_simplified(dist_cali, opt_k0);
  opt_k0 = dist_filtered;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_filtered:");
  Serial.println(dist_filtered);  
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
}
