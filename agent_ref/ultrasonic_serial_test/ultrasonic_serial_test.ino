/*
 * ================================================================
 * ultrasonic_serial_test.ino
 *
 * 獨立超音波 (HC-SR04) 測試程式 / Standalone HC-SR04 test sketch
 *
 * 用途 / Purpose:
 *   單純驗證超音波感測器是否正常,並在「電腦的序列埠監控視窗
 *   (Arduino IDE Serial Monitor)」直接看到距離讀值與會被發佈出去的內容。
 *   Verify the ultrasonic sensor and watch the distance reading directly
 *   on the computer's Serial Monitor.
 *
 * 重要 / IMPORTANT:
 *   這支程式「不使用 rosserial」,它用純文字 Serial.print 輸出。
 *   This sketch does NOT use rosserial; it prints plain text.
 *   - 同一個 USB 序列埠同一時間只能被一個程式佔用。測試時請「不要」
 *     在容器裡跑 rosserial 的 serial_node,否則會互搶序列埠。
 *     Do NOT run the rosserial serial_node on the same port while testing.
 *   - 測完後要回到正常運作,記得重新燒錄 CTRL_rosserial_tuned.ino。
 *     When done, re-flash CTRL_rosserial_tuned.ino to restore normal operation.
 *
 * 接腳 / Wiring (與正式韌體相同 / same as the real firmware):
 *   TRIG -> D11 , ECHO -> D12 , VCC -> 5V , GND -> GND
 *
 * 序列埠 / Serial Monitor:
 *   Baud = 115200 (Serial Monitor 右下角請選 115200)
 * ================================================================
 */

// ================= 超音波 (HC-SR04) 腳位定義 =================
#define Pin_Ultrasonic_Trig 11   // 觸發腳 (TRIG)
#define Pin_Ultrasonic_Echo 12   // 回波腳 (ECHO)

// ================= 量測參數 (與正式韌體一致) =================
const unsigned long ECHO_TIMEOUT_US = 25000;   // pulseIn 逾時 (微秒),約對應 4.2m 量程上限
const float SOUND_CM_PER_US = 0.01715;         // 343m/s,來回除2 → us * 0.0343 / 2

// ================= 輸出排程 =================
const unsigned long printIntervalMs = 200;     // 每 200ms 印一次 (5Hz),方便閱讀;正式韌體是 100ms
unsigned long previousPrintMillis = 0;

// 觸發一次 HC-SR04,回傳回波時間 (微秒);量不到回波 (逾時) 回傳 0
unsigned long readEchoDurationUs() {
  digitalWrite(Pin_Ultrasonic_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Pin_Ultrasonic_Trig, HIGH);
  delayMicroseconds(10);                 // 10us 觸發脈衝
  digitalWrite(Pin_Ultrasonic_Trig, LOW);

  return pulseIn(Pin_Ultrasonic_Echo, HIGH, ECHO_TIMEOUT_US);
}

void setup() {
  pinMode(Pin_Ultrasonic_Trig, OUTPUT);
  pinMode(Pin_Ultrasonic_Echo, INPUT);
  digitalWrite(Pin_Ultrasonic_Trig, LOW);

  Serial.begin(115200);
  while (!Serial) { ; }  // 等待序列埠就緒 (Mega 上會立即通過)

  Serial.println(F("==============================================="));
  Serial.println(F(" HC-SR04 Ultrasonic Serial Test (no rosserial)"));
  Serial.println(F(" TRIG=D11  ECHO=D12  |  Serial Monitor @115200"));
  Serial.println(F(" dist = 距離(cm) | echo = 回波時間(us) | publish = 會送到 /ultrasonic 的值"));
  Serial.println(F("==============================================="));
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousPrintMillis >= printIntervalMs) {
    previousPrintMillis = currentMillis;

    unsigned long duration = readEchoDurationUs();

    // 與正式韌體 readUltrasonicCm() 相同邏輯:逾時送 -1.0 代表超出量程
    // Same logic as the firmware: timeout -> publish -1.0 (out of range)
    float distance_cm = (duration == 0) ? -1.0 : (duration * SOUND_CM_PER_US);

    if (duration == 0) {
      Serial.print(F("dist = OUT OF RANGE   | echo = timeout    | publish(/ultrasonic).data = "));
      Serial.println(distance_cm, 2);     // 會印出 -1.00
    } else {
      Serial.print(F("dist = "));
      Serial.print(distance_cm, 2);
      Serial.print(F(" cm | echo = "));
      Serial.print(duration);
      Serial.print(F(" us | publish(/ultrasonic).data = "));
      Serial.println(distance_cm, 2);
    }
  }
}
