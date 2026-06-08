#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>   // 超音波距離發佈用

// ================= 編碼器腳位定義 (已更新) =================
#define Pin_Encoder_Right_A 2          
#define Pin_Encoder_Right_B 20        
#define Pin_Encoder_Left_A 3         
#define Pin_Encoder_Left_B 21          

volatile long theta_Right = 0;
volatile long theta_Left = 0;

// ================= PID 與時間計算變數 =================
unsigned long previousMillis = 0;
float interval = 50.0; // PID 控制週期 (毫秒)，50ms = 20Hz

// ================= 斷線保護 (Command Watchdog) =================
// 上位機正常情況下以 20Hz (每 50ms) 下達速度指令。若超過此門檻仍未收到新指令，
// 視為通訊中斷 (Ctrl+C / 拔線 / 上位機 crash)，強制歸零目標速度讓車子停下。
const unsigned long CMD_TIMEOUT_MS = 300; // 約 6 個控制週期
volatile unsigned long lastCmdMillis = 0; // 最後一次收到速度指令的時間戳

// 馬達物理參數
const float CPR = 330.0; 

// 低通濾波器參數
float filtered_rpm_L = 0.0;
float filtered_rpm_R = 0.0;
const float alpha = 0.2; 

// PID 參數區 (您調好的完美參數)
// float Kp_L = 1.1, Ki_L = 4.0, Kd_L = 0;
// float Kp_R = 1.1, Ki_R = 4.0, Kd_R = 0;

float Kp_L = 0.7, Ki_L = 1.5, Kd_L = 0;
float Kp_R = 0.7, Ki_R = 1.5, Kd_R = 0;

// PID 狀態變數 (RPM)
float target_rpm_L = 0.0, target_rpm_R = 0.0;
float current_rpm_L = 0.0, current_rpm_R = 0.0;
float error_L = 0, last_error_L = 0, integral_L = 0;
float error_R = 0, last_error_R = 0, integral_R = 0;
float pwm_L = 0, pwm_R = 0;

long last_theta_Left = 0;
long last_theta_Right = 0;

// ================= 控制器腳位定義 =================
#define PWMA 4
#define AIN1 6
#define AIN2 5
#define STBY 7
#define PWMB 8
#define BIN1 9
#define BIN2 10

// ================= 超音波 (HC-SR04) 腳位定義 =================
#define Pin_Ultrasonic_Trig 11   // 觸發腳 (TRIG)
#define Pin_Ultrasonic_Echo 12   // 回波腳 (ECHO)

// 超音波量測排程與參數 (與 PID 分開排程，避免阻塞干擾控制迴圈)
unsigned long previousUltrasonicMillis = 0;
const float ultrasonicInterval = 100.0;        // 量測週期 (毫秒)，100ms = 10Hz
const unsigned long ECHO_TIMEOUT_US = 25000;   // pulseIn 逾時 (微秒)，約對應 4.2m 量程上限

// ================= ROS Node 與回呼函式 =================
ros::NodeHandle nh;

// 比例乘數 (現在這兩個數值是用來將 ROS 的速度 m/s 轉換為「目標 RPM」，需依據實車輪徑/輪距微調)
const float LINEAR_SCALE = 289.4;
const float ANGULAR_SCALE = 23.8; 

void velCallback(const geometry_msgs::Twist& vel_msg) {
  // 從 Twist 中提取線速度(x)與角速度(z)
  float linear_x = vel_msg.linear.x;
  float angular_z = vel_msg.angular.z;

  // 將 ROS 下達的速度轉換為左右輪的「目標 RPM」
  target_rpm_L = (linear_x * LINEAR_SCALE) - (angular_z * ANGULAR_SCALE);
  target_rpm_R = (linear_x * LINEAR_SCALE) + (angular_z * ANGULAR_SCALE);

  // 餵狗：記錄收到指令的時間，供 watchdog 判斷通訊是否中斷
  lastCmdMillis = millis();
}

// 建立訂閱者 (Subscriber)
ros::Subscriber<geometry_msgs::Twist> sub("arduino_vel", velCallback);

// 建立超音波距離發佈者 (Publisher)，單位 cm；量不到回波時送 -1.0 代表超出量程
std_msgs::Float32 ultrasonic_msg;
ros::Publisher pub_ultrasonic("ultrasonic", &ultrasonic_msg);

// ================= 硬體初始化函式 =================
void initMotor() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT); pinMode(STBY, OUTPUT); 
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH); analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void initEncoder() {
  pinMode(Pin_Encoder_Right_A, INPUT_PULLUP); pinMode(Pin_Encoder_Right_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Right_A), doEncoder_Right, RISING);
  pinMode(Pin_Encoder_Left_A, INPUT_PULLUP); pinMode(Pin_Encoder_Left_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pin_Encoder_Left_A), doEncoder_Left, RISING);  
}

void initUltrasonic() {
  pinMode(Pin_Ultrasonic_Trig, OUTPUT);
  pinMode(Pin_Ultrasonic_Echo, INPUT);
  digitalWrite(Pin_Ultrasonic_Trig, LOW);
}

// 觸發一次 HC-SR04 並回傳距離 (cm)，量不到回波 (逾時) 回傳 -1.0
float readUltrasonicCm() {
  digitalWrite(Pin_Ultrasonic_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Pin_Ultrasonic_Trig, HIGH);
  delayMicroseconds(10);                 // 10us 觸發脈衝
  digitalWrite(Pin_Ultrasonic_Trig, LOW);

  unsigned long duration = pulseIn(Pin_Ultrasonic_Echo, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0;        // 逾時 / 超出量程
  return duration * 0.01715;             // 音速 343m/s，來回除2 → duration(us) * 0.0343 / 2
}

void SetPWM(int motor, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (motor == 1) { 
    if (pwm >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, pwm); } 
    else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -pwm); }
  } else if (motor == 2) { 
    if (pwm >= 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm); } 
    else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, -pwm); }
  }
}

// 可當作斷線保護預留使用
void stopMove() {
  target_rpm_L = 0;
  target_rpm_R = 0;
}

// ================= 主程式 =================
void setup() {
  initMotor();
  initEncoder();
  initUltrasonic();

  // 初始化 ROS
  nh.getHardware()->setBaud(115200); // 確保 rosserial 鮑率符合您的設定
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_ultrasonic);
}

void loop() {
  // 維持 ROS 通訊的核心函式
  nh.spinOnce();

  unsigned long currentMillis = millis();

  // ====== 斷線保護 Watchdog ======
  // 超過 CMD_TIMEOUT_MS 未收到新速度指令 (或 rosserial 已斷線) 就強制停車。
  // 目標歸零後，下方 PID 迴圈會自動清掉積分項並把 PWM 設為 0。
  if ((currentMillis - lastCmdMillis > CMD_TIMEOUT_MS) || !nh.connected()) {
    stopMove();
  }

  // ====== PID 控制迴圈 (每 50 毫秒執行一次) ======
  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - previousMillis) / 1000.0; 
    previousMillis = currentMillis;

    long current_theta_L = theta_Left;
    long current_theta_R = theta_Right;

    float ticks_per_sec_L = (current_theta_L - last_theta_Left) / dt;
    float ticks_per_sec_R = (current_theta_R - last_theta_Right) / dt;

    float raw_rpm_L = (ticks_per_sec_L * 60.0) / CPR;
    float raw_rpm_R = (ticks_per_sec_R * 60.0) / CPR;

    // 套用一階低通濾波
    filtered_rpm_L = (alpha * raw_rpm_L) + ((1.0 - alpha) * filtered_rpm_L);
    filtered_rpm_R = (alpha * raw_rpm_R) + ((1.0 - alpha) * filtered_rpm_R);

    current_rpm_L = filtered_rpm_L;
    current_rpm_R = filtered_rpm_R;

    last_theta_Left = current_theta_L;
    last_theta_Right = current_theta_R;

    // --- 左輪 PID ---
    if (target_rpm_L == 0) {
      pwm_L = 0; integral_L = 0; last_error_L = 0; 
    } else {
      error_L = target_rpm_L - current_rpm_L;
      integral_L += (error_L * dt);
      integral_L = constrain(integral_L, -500, 500); 
      float derivative_L = (error_L - last_error_L) / dt;
      pwm_L = (Kp_L * error_L) + (Ki_L * integral_L) + (Kd_L * derivative_L);
      last_error_L = error_L;
    }

    // --- 右輪 PID ---
    if (target_rpm_R == 0) {
      pwm_R = 0; integral_R = 0; last_error_R = 0;
    } else {
      error_R = target_rpm_R - current_rpm_R;
      integral_R += (error_R * dt);
      integral_R = constrain(integral_R, -500, 500); 
      float derivative_R = (error_R - last_error_R) / dt;
      pwm_R = (Kp_R * error_R) + (Ki_R * integral_R) + (Kd_R * derivative_R);
      last_error_R = error_R;
    }

    SetPWM(1, (int)pwm_L);
    SetPWM(2, (int)pwm_R);
  }

  // ====== 超音波量測與發佈 (每 100 毫秒執行一次) ======
  if (currentMillis - previousUltrasonicMillis >= ultrasonicInterval) {
    previousUltrasonicMillis = currentMillis;
    ultrasonic_msg.data = readUltrasonicCm();   // 距離 (cm)，-1.0 = 超出量程
    pub_ultrasonic.publish(&ultrasonic_msg);
  }
}

// ================= 編碼器中斷服務常式 =================
#ifdef ARDUINO_ARCH_ESP32
  #define INTERRUPT_ATTR IRAM_ATTR
#else
  #define INTERRUPT_ATTR
#endif

INTERRUPT_ATTR void doEncoder_Right() {
  if (digitalRead(Pin_Encoder_Right_B) == LOW) theta_Right--; else theta_Right++;
}

INTERRUPT_ATTR void doEncoder_Left() {
  if (digitalRead(Pin_Encoder_Left_B) == LOW) theta_Left++; else theta_Left--;
}