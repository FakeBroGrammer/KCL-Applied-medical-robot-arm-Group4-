// ==================== Motor 1 (大电机 m1) ====================

const int outputA_m1 = 2;
const int outputB_m1 = 10;

const int mA_in1 = 4;
const int mA_in2 = 5;
const int mA_en  = 6;

// 大电机编码器参数
const float PPR_m1 = 6.68;   // 每圈脉冲数（你测出来的）
const float GR_m1  = 30;     // 减速比

// 状态变量
volatile long counter_m1 = 0;   // 中断里更新的脉冲计数
int aLastState_m1;              // 上一次 A 相状态
float currentDegrees_m1 = 0.0;
float targetDegrees_m1  = 0.0; // 大电机目标角度

// 时间控制
const float dt_m1           = 0.01;   // 10ms
const unsigned long dt_ms_m1 = 10;
unsigned long lastTime_m1   = 0;

// PID (EMG30 )
float Kp_m1 = 3.45;
float Ki_m1 = 10;
float Kd_m1 = 0.13;

float error_m1     = 0.0;
float errorPrev_m1 = 0.0;
float errorSum_m1  = 0.0;
float errorDiff_m1 = 0.0;
float u_m1         = 0.0;


// ==================== Motor 2 (小电机 m2) ====================

const int outputC_m2 = 3;
const int outputD_m2 = 11;

const int mB_in3 = 7;
const int mB_in4 = 8;
const int mB_en  = 9;

// 小电机编码器参数
const float PPR_m2 = 5.861;   // 每圈脉冲数（你测的）
const float GR_m2  = 250;     // 减速比

// 状态变量
volatile long counter_m2 = 0;
int aLastState_m2;
float currentDegrees_m2 = 0.0;
float targetDegrees_m2  = 0.0;  // 小电机目标角度

// 时间控制
const float dt_m2           = 0.01;  // 10ms
const unsigned long dt_ms_m2 = 10;
unsigned long lastTime_m2   = 0;

// PID (small motor)
float Kp_m2 = 7.6;
float Ki_m2 = 4.5;    //6.85
float Kd_m2 = 0.2;    //0.48

float error_m2     = 0.0;
float errorPrev_m2 = 0.0;
float errorSum_m2  = 0.0;
float errorDiff_m2 = 0.0;
float u_m2         = 0.0;


// ==================== 通用：初始化 ====================

void setup() {
  Serial.begin(9600);

  // --- 大电机 m1 编码器 & 电机 ---
  pinMode(outputA_m1, INPUT_PULLUP);
  pinMode(outputB_m1, INPUT_PULLUP);

  pinMode(mA_in1, OUTPUT);
  pinMode(mA_in2, OUTPUT);
  pinMode(mA_en,  OUTPUT);

  aLastState_m1 = digitalRead(outputA_m1);
  attachInterrupt(digitalPinToInterrupt(outputA_m1), updateEncoder_m1, CHANGE);

  digitalWrite(mA_in1, HIGH);
  digitalWrite(mA_in2, LOW);
  analogWrite(mA_en, 0);

  lastTime_m1 = millis();

  // --- 小电机 m2 编码器 & 电机 ---
  pinMode(outputC_m2, INPUT_PULLUP);
  pinMode(outputD_m2, INPUT_PULLUP);

  pinMode(mB_in3, OUTPUT);
  pinMode(mB_in4, OUTPUT);
  pinMode(mB_en,  OUTPUT);

  aLastState_m2 = digitalRead(outputC_m2);
  attachInterrupt(digitalPinToInterrupt(outputC_m2), updateEncoder_m2, CHANGE);

  digitalWrite(mB_in3, HIGH);
  digitalWrite(mB_in4, LOW);
  analogWrite(mB_en, 0);

  lastTime_m2 = millis();



}


void readTargetsFromSerial() {
  if (Serial.available() <= 0) return;

  String data = Serial.readStringUntil('\n');
  data.trim();

  int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      targetDegrees_m1 = data.substring(0, commaIndex).toFloat();
      targetDegrees_m2 = data.substring(commaIndex + 1).toFloat();
    }
}


// ==================== 主循环：两个电机各自用自己的 dt ====================

void loop() {

  readTargetsFromSerial();
  
  unsigned long now = millis();

  // ---------- 大电机 m1 控制 ----------
  if (now - lastTime_m1 >= dt_ms_m1) {
    lastTime_m1 += dt_ms_m1;

    long counts_m1;
    noInterrupts();
    counts_m1 = counter_m1;
    interrupts();

    // counts -> 角度
    currentDegrees_m1 = ((float)counts_m1 * 360.0) / (PPR_m1 * GR_m1);

    // 防止角度飘得太远
    if (currentDegrees_m1 > 360.0 || currentDegrees_m1 < -360.0) {
      noInterrupts();
      counter_m1 = 0;
      counts_m1  = 0;
      interrupts();
      currentDegrees_m1 = 0.0;
      errorPrev_m1 = 0;
      errorSum_m1  = 0;
    }

    // PID error
    error_m1     = targetDegrees_m1 - currentDegrees_m1;
    errorDiff_m1 = (error_m1 - errorPrev_m1) / dt_m1;
    errorSum_m1 += error_m1;

    // PID output
    u_m1 = error_m1 * Kp_m1 + errorDiff_m1 * Kd_m1 + errorSum_m1 * Ki_m1 * dt_m1;

    // 限幅
    if (u_m1 > 255)  u_m1 = 255;
    if (u_m1 < -255) u_m1 = -255;

    // 死区：误差很小时停住
    if (fabs(error_m1) < 0.5) {
      u_m1 = 0;
      errorSum_m1 = 0;
    }

    // 输出到大电机
    if (u_m1 >= 0) {
      digitalWrite(mA_in1, HIGH);
      digitalWrite(mA_in2, LOW);
      analogWrite(mA_en, (int)round(u_m1));
    } else {
      digitalWrite(mA_in1, LOW);
      digitalWrite(mA_in2, HIGH);
      analogWrite(mA_en, (int)round(-u_m1));
    }

    errorPrev_m1 = error_m1;
  }

  // ---------- 小电机 m2 控制 ----------
  if (now - lastTime_m2 >= dt_ms_m2) {
    lastTime_m2 += dt_ms_m2;

    long counts_m2;
    noInterrupts();
    counts_m2 = counter_m2;
    interrupts();

    currentDegrees_m2 = ((float)counts_m2 * 360.0) / (PPR_m2 * GR_m2);

    if (currentDegrees_m2 > 360.0 || currentDegrees_m2 < -360.0) {
      noInterrupts();
      counter_m2 = 0;
      counts_m2  = 0;
      interrupts();
      currentDegrees_m2 = 0.0;
      errorPrev_m2 = 0;
      errorSum_m2  = 0;
    }

    error_m2     = targetDegrees_m2 - currentDegrees_m2;
    errorDiff_m2 = (error_m2 - errorPrev_m2) / dt_m2;
    errorSum_m2 += error_m2;

    u_m2 = error_m2 * Kp_m2 + errorDiff_m2 * Kd_m2 + errorSum_m2 * Ki_m2 * dt_m2;

    if (u_m2 > 255)  u_m2 = 255;
    if (u_m2 < -255) u_m2 = -255;

    if (fabs(error_m2) < 0.5) {
      u_m2 = 0;
      errorSum_m2 = 0;
    }

    if (u_m2 >= 0) {
      digitalWrite(mB_in3, HIGH);
      digitalWrite(mB_in4, LOW);
      analogWrite(mB_en, (int)round(u_m2));
    } else {
      digitalWrite(mB_in3, LOW);
      digitalWrite(mB_in4, HIGH);
      analogWrite(mB_en, (int)round(-u_m2));
    }

    errorPrev_m2 = error_m2;
  }

  // ------ 串口输出（4列：m1当前 m1目标 m2当前 m2目标）------
  Serial.print(currentDegrees_m1);
  Serial.print(" ");
  Serial.print(targetDegrees_m1);
  Serial.print(" ");
  Serial.print(currentDegrees_m2);
  Serial.print(" ");
  Serial.println(targetDegrees_m2);
}




void updateEncoder_m1() {
  int aState_m1 = digitalRead(outputA_m1);
  int bState_m1 = digitalRead(outputB_m1);

  if (aState_m1 != aLastState_m1) {
    if (aState_m1 != bState_m1) {
      counter_m1++;      
    } else {
      counter_m1--;
    }
  }

  aLastState_m1 = aState_m1;
}




void updateEncoder_m2() {
  int aState_m2 = digitalRead(outputC_m2);
  int bState_m2 = digitalRead(outputD_m2);

  if (aState_m2 != aLastState_m2) {
    if (aState_m2 != bState_m2) {
      counter_m2--;      
    } else {
      counter_m2++;      
    }
  }

  aLastState_m2 = aState_m2;
}