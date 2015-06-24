int steering, throttle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  steering = throttle = 0;  
}

void loop() {
  delay(33); // Run at around 30 Hz.

  steering += 5;
  if (steering > 180)
    steering = -15;
  
  throttle += 3;
  if (throttle > 179)
    throttle = -179;
    
  Serial.print("<");
  Serial.print(steering);
  Serial.print(",");
  Serial.print(throttle);
  Serial.print(">");
}
