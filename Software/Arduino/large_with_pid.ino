/*
   7MRI0060 - Applied Medical Robotics Module
   September 2024
   Author: Harry Robertshaw

   Purpose: Read encoder with interrupt and measure motor angle with PID control
*/

// Define the pins connected to encoder channels A and B
#define outputA 2
#define outputB 10

const int mA_in1 = 4;
const int mA_in2 = 5;
const int mA_en = 6;

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const int PPR = 180;  // Pulses per revolution of the encoder
const float GR = 1;   // Gear ratio

// PID 参数
double Kp = 1.1;   // 比例增益 - 需要根据实际调整
double Ki = 0.05;   // 积分增益 - 需要根据实际调整  
double Kd = 0.0;  // 微分增益 - 需要根据实际调整

// PID 变量
double setpoint = 90.0;     // 目标角度 (度)
double input = 0.0;         // 反馈角度 (度)
double output = 0.0;        // PID输出
double error = 0.0;         // 误差
double lastError = 0.0;     // 上一次误差
double integral = 0.0;      // 积分项
double derivative = 0.0;    // 微分项

// 时间控制
unsigned long lastTime = 0;
unsigned long stableTime = 0;  // 移到全局变量区域
double deltaTime = 0.0;

// 输出限制
const int OUTPUT_MIN = -255;
const int OUTPUT_MAX = 255;
const int PWM_MIN = 40;     // 最小PWM值，确保电机能启动

// Declare variables
int aLastState;             // Previous state of channel A
float positionInDegrees_m2 = 0.0; // Position in degrees for motor 2
int position_m2 = 0;        // Encoder position for motor 2
long targetCounts = 0;

volatile long counter = 0; 
void updateEncoder();
void computePID();
void setMotorSpeed(int speed);

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

  // 计算目标计数
  long countsPerOutputRev = (long)PPR * (long)GR;
  targetCounts = (countsPerOutputRev * 90) / 360;

  Serial.print("countsPerOutputRev = ");
  Serial.println(countsPerOutputRev);
  Serial.print("targetCounts (for 90 deg) = ");
  Serial.println(targetCounts);
  Serial.println("PID Control Started");

  lastTime = millis();
  stableTime = 0;  // 初始化稳定计时器

  // noInterrupts();
  // counter = 0;
  // interrupts();
}

void loop() {
  // 计算当前角度
  positionInDegrees_m2 = ((float)position_m2 * 360.0) / (PPR * GR);
  input = positionInDegrees_m2;
  
  // 计算PID
  computePID();
  
  // 应用PID输出到电机
  setMotorSpeed((int)output);
  
  // 打印调试信息
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" | Current: ");
  Serial.print(input, 1);
  Serial.print(" | Error: ");
  Serial.print(error, 1);
  Serial.print(" | Output: ");
  Serial.println(output);
  
  // 检查是否到达目标位置（带容差）
  if (fabs(error) < 2.0) {  // 2度容差
    if (stableTime == 0) {
      stableTime = millis();
    }
    // 稳定一段时间后才停止
    if (millis() - stableTime > 500) {
      analogWrite(mA_en, 0);
      digitalWrite(mA_in1, LOW);
      digitalWrite(mA_in2, LOW);
      Serial.println("Reached target position with PID. Motor stopped.");
      
      while (1) {
        noInterrupts();
        long c = counter;
        interrupts();
        float deg = ((float)c * 360.0) / ((float)PPR * GR);
        Serial.print("Final Position (deg): ");
        Serial.println(deg, 2);
        delay(1000);
      }
    }
  } else {
    stableTime = 0;  // 重置稳定计时器
  }
  
  delay(50);  // 更快的控制周期
}

void computePID() {
  unsigned long currentTime = millis();
  deltaTime = (double)(currentTime - lastTime) / 1000.0;  // 转换为秒
  
  if (deltaTime == 0) return;  // 避免除以零
  
  error = setpoint - input;
  
  // 比例项
  double proportional = Kp * error;
  
  // 积分项（带抗饱和）
  integral += error * deltaTime;
  // 积分限幅
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  double integralTerm = Ki * integral;
  
  // 微分项
  derivative = (error - lastError) / deltaTime;
  double derivativeTerm = Kd * derivative;
  
  // 计算PID输出
  output = proportional + integralTerm + derivativeTerm;
  
  // 输出限幅
  if (output > OUTPUT_MAX) output = OUTPUT_MAX;
  if (output < OUTPUT_MIN) output = OUTPUT_MIN;
  
  lastError = error;
  lastTime = currentTime;
}

void setMotorSpeed(int speed) {
  // 确保最小PWM值以克服静摩擦力
  int pwm = abs(speed);
  if (pwm > 0 && pwm < PWM_MIN) {
    pwm = PWM_MIN;
  }
  
  if (speed > 0) {
    // 正转
    digitalWrite(mA_in1, HIGH);
    digitalWrite(mA_in2, LOW);
    analogWrite(mA_en, pwm);
  } else if (speed < 0) {
    // 反转
    digitalWrite(mA_in1, LOW);
    digitalWrite(mA_in2, HIGH);
    analogWrite(mA_en, pwm);
  } else {
    // 停止
    digitalWrite(mA_in1, LOW);
    digitalWrite(mA_in2, LOW);
    analogWrite(mA_en, 0);
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