#define outputC 3
#define outputD 11

const int mB_in3 = 7;
const int mB_in4 = 8;
const int mB_en = 9;

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR_m2 = 5.861 ;  // Pulses per revolution of the encoder
const float GR_m2 = 250; // Gear ratio
// Declare variables

int aLastState_m2;       // Previous state of channel A
int position_m2 = 0;  // Encoder position for motor 2
float targetDegrees = 30;
float currentDegrees = 0.0;

volatile long counter = 0;
int aLastState;





const float dt = 0.01;     // PID 用的固定采样周期 = 10ms
const unsigned long dt_ms = 10;  
unsigned long lastTime = 0;  


// PID
float Kp_m2 = 8.5;
float Ki_m2 = 0.85;
float Kd_m2 = 0.4;



float error      = 0.0;
float errorPrev  = 0.0;
float errorSum   = 0.0;
float errorDiff  = 0.0;
float u          = 0.0; 

void setup() {
  // Setup pin modes for the encoder pins
  pinMode(outputC, INPUT_PULLUP);
  pinMode(outputD, INPUT_PULLUP);

  pinMode(mB_in3, OUTPUT);
  pinMode(mB_in4, OUTPUT);
  pinMode(mB_en, OUTPUT);

  // Start serial communication at 9600 baud
  Serial.begin(9600);
  // Read the initial state of channel A
  aLastState = digitalRead(outputC);
  // Attach an interrupt to detect changes on channel A
  attachInterrupt(digitalPinToInterrupt(outputC), updateEncoder, CHANGE);



  digitalWrite(mB_in3, HIGH);
  digitalWrite(mB_in4, LOW);
  analogWrite(mB_en, 0);

  lastTime = millis();

}
void loop() {
  unsigned long now = millis();

  if (now - lastTime >= dt_ms) {
    lastTime += dt_ms;
    long counts;
    noInterrupts();
    counts = counter;
    interrupts();

  


  currentDegrees = ( (float)position_m2 * 360.0 ) / (PPR_m2 * GR_m2);
  // Transform position to degrees for motor 2
  // INSERT CODE HERE
  // Print the encoder position in degrees
  if (currentDegrees > 360.0 || currentDegrees < -360.0) {
      noInterrupts();
      counter = 0;
      counts = 0;
      interrupts();
      currentDegrees = 0.0;
    }

    error     = targetDegrees - currentDegrees;
    errorDiff = (error - errorPrev) / dt;
    errorSum += error;

    // 5. PID 控制律
    u = error * Kp_m2 + errorDiff * Kd_m2 + errorSum * Ki_m2 * dt;

    if (u > 255)  u = 255;
    if (u < -255) u = -255;

    // 小死区：误差很小就认为到位，停住，防止抖动
    if (fabs(error) < 0.5) { //死区可改。0.5
      u = 0;
      errorSum = 0;
    }
    if (u >= 0) {
      // 正方向
      digitalWrite(mB_in3, HIGH);
      digitalWrite(mB_in4, LOW);
      analogWrite(mB_en, (int)round(u));
    } else {
      // 反方向
      digitalWrite(mB_in3, LOW);
      digitalWrite(mB_in4, HIGH);
      analogWrite(mB_en, (int)round(-u));
    }
    errorPrev = error;

    // 9. 输出给 Serial Plotter：当前角度 & 目标角度
    // 在 Tools → Serial Plotter 里即可看到两条曲线
    Serial.print(currentDegrees);
    Serial.print(" ");
    Serial.println(targetDegrees);
  }
}



  // Check if the degree value is outside the range {-360, 360} and reset the position if it exceeds 360 degrees or drops below -360 degre
  

// Interrupt Service Routine (ISR) - This function is called whenever channel A changes state
void updateEncoder() {
  int aState = digitalRead(outputC);  // Read current state of channel A
  int bState = digitalRead(outputD);  // Read current state of channel B
  // If the state of channel A has changed
  if (aState != aLastState) {
    // Check the direction of rotation by comparing A and B
    if (aState != bState) {
      counter--;  // Clockwise rotation
    } else {
      counter++;  // Counterclockwise rotation
    }
    position_m2 = counter; 
    // Update position for motor 2 based on encoder count
  }
  // Update the last known state of channel A
  aLastState = aState;
}
