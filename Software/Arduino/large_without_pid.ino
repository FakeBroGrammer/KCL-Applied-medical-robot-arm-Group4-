/*
   7MRI0060 - Applied Medical Robotics Module
   September 2024
   Author: Harry Robertshaw

   Purpose: Read encoder with interrupt and measure motor angle   不加PID的大电机代码
*/

// Define the pins connected to encoder channels A and B
#define outputA 2
#define outputB 10

const int mA_in1 = 4;
const int mA_in2 = 5;
const int mA_en = 6;

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const int PPR = 180 ;  // Pulses per revolution of the encoder
const float GR = 1; // Gear ratio
// Declare variables

int aLastState;       // Previous state of channel A
float positionInDegrees_m2 = 0.0; // Position in degrees for motor 2
int position_m2 = 0;  // Encoder position for motor 2
long targetCounts = 0;

volatile long counter = 0; 
void updateEncoder();


void setup() {
  // Setup pin modes for the encoder pins
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);

  pinMode(mA_in1, OUTPUT);
  pinMode(mA_in2, OUTPUT);
  pinMode(mA_en, OUTPUT);

  // Start serial communication at 9600 baud
  Serial.begin(9600);
  // Read the initial state of channel A
  aLastState = digitalRead(outputA);
  // Attach an interrupt to detect changes on channel A
  attachInterrupt(digitalPinToInterrupt(outputA), updateEncoder, CHANGE);


  long countsPerOutputRev = (long)PPR * (long)GR;
  //60度对应的计数
  targetCounts = (countsPerOutputRev * 90) / 360; // 整数运算
  // 若希望更精确可用浮点，但整数足够用于比较

  Serial.print("countsPerOutputRev = ");
  Serial.println(countsPerOutputRev);
  Serial.print("targetCounts (for 90 deg) = ");
  Serial.println(targetCounts);

  digitalWrite(mA_in1, HIGH);
  digitalWrite(mA_in2, LOW);
  analogWrite(mA_en, 59);

}
void loop() {
  positionInDegrees_m2 = ( (float)position_m2 * 360.0 ) / (PPR * GR);
  // Transform position to degrees for motor 2
  // INSERT CODE HERE
  // Print the encoder position in degrees
  Serial.print("Position in Degrees (Motor 2): ");
  Serial.println(positionInDegrees_m2);
  if (labs(position_m2) >= labs(targetCounts)) {
    // 停止电机（将 IN1/IN2 拉低并关闭 PWM）
    analogWrite(mA_en, 0);
    digitalWrite(mA_in1, LOW);
    digitalWrite(mA_in2, LOW);
    Serial.println("Reached 90 degrees (or exceeded). Motor stopped.");
    while (1) {
      noInterrupts();
      long c = counter;
      interrupts();
      float deg = ((float)c * 360.0) / ((float)PPR * GR);
      Serial.print("Final Position (deg): ");
      Serial.println(deg, 2);
      delay(500);
    }
  // Check if the degree value is outside the range {-360, 360} and reset the position if it exceeds 360 degrees or drops below -360 degrees
  // INSERT CODE HERE - HINT: use if statement to check positionInDegrees_m2, and then rearrange formula for position_m2
  delay(200); 
  }
}
// Interrupt Service Routine (ISR) - This function is called whenever channel A changes state
void updateEncoder() {
  int aState = digitalRead(outputA);  // Read current state of channel A
  int bState = digitalRead(outputB);  // Read current state of channel B
  // If the state of channel A has changed
  if (aState != aLastState) {
    // Check the direction of rotation by comparing A and B
    if (aState != bState) {
      counter++;  // Clockwise rotation
    } else {
      counter--;  // Counterclockwise rotation
    }
    position_m2 = counter; 
    // Update position for motor 2 based on encoder count
  }
  // Update the last known state of channel A
  aLastState = aState;
}