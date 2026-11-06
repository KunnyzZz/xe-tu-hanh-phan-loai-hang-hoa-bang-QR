// Firmware xe phân loại hàng trên đường HÌNH VUÔNG
// Lệnh HTTP: /status   /go?dest=A|B   /stop
//
// Đường đi: xe đặt ở vạch xuất phát (vạch ngang rộng), chạy bám line, rẽ các góc.
//   Điểm A / B: sau khi rẽ xong góc POINT_x_CORNER, xe bám line tới VẠCH NGANG của điểm đó,
//   dừng, servo dỡ hàng, rồi chạy tiếp rẽ nốt các góc còn lại tới vạch đích.
//   A và B phải nằm ở hai cạnh khác nhau. Vạch ngang của điểm kia nằm ở cạnh khác nên xe chỉ đi qua.
// Vật cản: HC-SR04 (TRIG 16, ECHO 17), gặp vật khi bám line thì dừng chờ, chờ quá lâu thì lỗi OBSTACLE.
// Mục có [TINH CHỈNH] là chỗ có thể sửa cho hợp xe (sửa xong phải nạp lại code).

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <stdarg.h>

// =====================================================================
//  MẤT KẾT NỐI (đặt ở đầu file cho dễ sửa)
// =====================================================================
const bool STOP_ON_NO_CONTACT = false;            // [TINH CHỈNH] false = mất kết nối vẫn chạy hết chuyến. true = dừng (lỗi NO_CONTACT).
const unsigned long HEARTBEAT_TIMEOUT_MS = 10000; // [TINH CHỈNH] Quá thời gian này không có lệnh từ server -> xe dừng (chỉ có tác dụng khi STOP_ON_NO_CONTACT = true).

// =====================================================================
//  THÔNG SỐ
// =====================================================================

// ---- WiFi (chỉ hỗ trợ 2.4 GHz) ----  [TINH CHỈNH] đổi tên/mật khẩu WiFi của bạn
const char* WIFI_SSID = "-.-";
const char* WIFI_PASS = "123456789";
const unsigned long WIFI_BOOT_WAIT_MS = 20000;  // Chờ WiFi lúc khởi động (ms). Mạng chậm: tăng.
const unsigned long WIFI_RECHECK_MS   = 5000;   // Mất WiFi thì thử nối lại sau mỗi bao lâu (ms).
const unsigned long IP_PRINT_MS       = 5000;   // [TINH CHỈNH] chu kỳ in địa chỉ IP ra Serial Monitor (ms).

// ---- Chân nối ----  [TINH CHỈNH] chỉ sửa nếu đấu dây khác
const int ENA = 33, IN1 = 25, IN2 = 26;
const int IN3 = 27, IN4 = 14, ENB = 13;
const int SENSOR_PINS[5] = {18, 19, 21, 22, 23};  // TRÁI -> PHẢI nhìn theo hướng xe chạy
const int PIN_SERVO = 4;
const int PIN_TRIG = 16;           // HC-SR04: chân TRIG
const int PIN_ECHO = 17;           // HC-SR04: chân ECHO (nhớ chia áp 5V -> 3.3V nếu module chạy 5V)
const bool BLACK_IS_HIGH = true;   // [TINH CHỈNH] cảm biến báo line đen là mức HIGH. Xe chạy ngược màu: đổi true/false.

// ---- Động cơ ----
const bool SWAP_MOTORS_LR = true;  // [TINH CHỈNH] xe quẹo ngược hướng khi lệch line: đổi true/false.
const int LEFT_MOTOR_GAIN  = 100;  // [TINH CHỈNH] bù lệch động cơ (%). Xe tự lệch phải: tăng LEFT hoặc giảm RIGHT.
const int RIGHT_MOTOR_GAIN = 100;  //              Xe tự lệch trái: làm ngược lại.

// ---- Lọc nhiễu cảm biến ----
const unsigned long SENSOR_DEBOUNCE_MS = 8;   // [TINH CHỈNH] cảm biến bị nhiễu/chớp: tăng. Phản ứng chậm: giảm.

// ---- Cú đẩy (kick): lúc xuất phát, sau mỗi lần rẽ xong, sau khi đổ hàng xong ----
const int   KICK_SPEED = 65;          // [TINH CHỈNH] tốc độ đẩy (%). Xe yếu, khựng khi chạy lại: tăng. Vọt khỏi line: giảm.
const unsigned long KICK_MS = 80;    // [TINH CHỈNH] thời gian đẩy (ms).

// ---- Bám line (PD, Ki = 0) ----
const int   SENSOR_WEIGHT[5] = {-5, -3, 0, 3, 5};  // [TINH CHỈNH] trọng số từng cảm biến; càng lớn càng bẻ lái mạnh.
const int   BASE_SPEED = 35;          // [TINH CHỈNH] tốc độ nền (%). Muốn chạy nhanh: tăng (nhớ tăng MAX_SPEED).
const int   MAX_SPEED  = 38;          // [TINH CHỈNH] tốc độ tối đa mỗi bánh. PHẢI lớn hơn BASE_SPEED.
const int   MIN_SPEED  = 0;           // Tốc độ tối thiểu mỗi bánh (0 = cho phép dừng hẳn bánh trong).
const int   BASE_MIN_SPEED = 25;      // [TINH CHỈNH] tốc độ nền thấp nhất khi vào cua (%).
const float SLOW_ON_CURVE = 2.0;      // [TINH CHỈNH] giảm tốc khi lệch line. Cua gắt hay văng: tăng.
const float KP = 45.0;                // [TINH CHỈNH] độ nhạy bẻ lái. Lắc qua lại: giảm. Bám chậm, bị trượt khỏi line: tăng.
const float KD = 0.1;                 // [TINH CHỈNH] giảm dao động. Vẫn lắc: tăng nhẹ. Giật cục: giảm.
const unsigned long PID_INTERVAL_MS = 10;  // Chu kỳ tính PD (ms). Thường không cần đổi.

// ---- Góc vuông ----
const unsigned long CORNER_CONFIRM_MS = 10;   // [TINH CHỈNH] nhận ra góc phải kéo dài bao lâu (ms). Báo góc nhầm: tăng.
const unsigned long CORNER_IGNORE_MS  = 700;  // [TINH CHỈNH] sau khi vào đoạn mới, bỏ qua dò góc trong ms này.
const int   NUDGE_SPEED = 30;                 // [TINH CHỈNH] tốc độ nhích tới trước khi thấy góc (%).
const unsigned long CORNER_NUDGE_MS  = 10;     // [TINH CHỈNH] thời gian nhích (ms). Xe rẽ sớm, tâm xe chưa tới góc: tăng.
const unsigned long CORNER_SETTLE_MS = 150;   // [TINH CHỈNH] dừng ổn định trước khi rẽ (ms). Xe còn trôi: tăng.
const int   TURN_SPEED      = 55;             // [TINH CHỈNH] tốc độ xoay giai đoạn đầu (%).
const int   TURN_SEEK_SPEED = 50;             // [TINH CHỈNH] tốc độ xoay lúc tìm line (%). Xoay lố line: giảm.
const unsigned long TURN_BLIND_MS = 90;      // [TINH CHỈNH] thời gian xoay "mù" (ms) để rời line cũ. Bắt nhầm line cũ: tăng.
const unsigned long TURN_CATCH_CONFIRM_MS = 10;  // Cảm biến giữa phải thấy line bao lâu thì coi là rẽ xong.
const unsigned long TURN_TIMEOUT_MS = 3000;   // Xoay quá lâu không thấy line thì báo lỗi TURN_FAIL.
const int   SQUARE_CORNERS = 4;               // Số góc của đường vuông.
const unsigned long STRAIGHT_MS = 300;        // [TINH CHỈNH] sau MỖI lần rẽ xong, đi thẳng thêm bao lâu (ms). 0 = tắt.
const int   STRAIGHT_SPEED = 40;              // [TINH CHỈNH] tốc độ đi thẳng (%). Xe yếu, khựng: tăng. Xe lệch khỏi line: giảm.

// ---- ĐIỂM A / B (chỗ dỡ hàng, nhận biết bằng vạch ngang) ----
const int  POINT_A_CORNER = 1;                // [TINH CHỈNH] điểm A nằm sau góc thứ mấy.
const int  POINT_B_CORNER = 2;                // [TINH CHỈNH] điểm B nằm sau góc thứ mấy (khác POINT_A_CORNER).
const int  POINT_MARK_COUNT = 4;              // [TINH CHỈNH] số cảm biến thấy đen cùng lúc để coi là vạch ngang.
const unsigned long POINT_CONFIRM_MS = 15;    // Thấy vạch ngang kéo dài bao lâu thì dừng (ms).
const unsigned long POINT_IGNORE_MS  = 100;   // Bỏ qua dò vạch ngang trong ms đầu của đoạn.
const unsigned long POINT_MAX_MS     = 8000;  // [TINH CHỈNH] quá thời gian này không thấy vạch -> lỗi POINT_MISSED.

// ---- Servo dỡ hàng ----
const int SERVO_REST_ANGLE  = 0;              // [TINH CHỈNH] góc servo lúc nghỉ (độ).
const int SERVO_PUSH_ANGLE  = 90;             // [TINH CHỈNH] góc servo lúc đẩy hàng. Hàng chưa rớt hết: tăng.
const unsigned long SERVO_PUSH_MS   = 300;    // [TINH CHỈNH] chờ servo đẩy tới nơi (ms).
const unsigned long SERVO_HOLD_MS   = 800;    // [TINH CHỈNH] giữ servo để hàng rớt (ms). Hàng kẹt: tăng.
const unsigned long SERVO_RETURN_MS = 600;    // [TINH CHỈNH] chờ servo về vị trí nghỉ (ms).

// ---- VẠCH ĐÍCH ----
const bool  HOME_USE_MARK = true;             // [TINH CHỈNH] true = dừng khi thấy vạch rộng; false = chạy HOME_RUN_MS rồi dừng.
const int   HOME_MARK_COUNT = 4;              // [TINH CHỈNH] số cảm biến thấy đen cùng lúc để coi là vạch đích.
const unsigned long HOME_CONFIRM_MS = 15;     // Thấy vạch đích kéo dài bao lâu thì dừng (ms).
const unsigned long HOME_IGNORE_MS  = 300;    // Bỏ qua dò vạch đích trong ms đầu của đoạn cuối.
const unsigned long HOME_MAX_MS     = 8000;   // [TINH CHỈNH] quá thời gian này không thấy vạch -> lỗi HOME_MISSED.
const unsigned long HOME_RUN_MS     = 1500;   // Chỉ dùng khi HOME_USE_MARK = false.

// ---- Vật cản (HC-SR04) ----
// Gặp vật cản khi đang bám line -> xe DỪNG chờ; vật được dọn đi -> chạy tiếp.
// Chờ quá OBSTACLE_MAX_WAIT_MS -> lỗi OBSTACLE. Không kiểm tra lúc xoay góc hay dỡ hàng.
const bool  OBSTACLE_ENABLED = true;          // [TINH CHỈNH] false = tắt hẳn tránh vật cản.
const float OBSTACLE_DIST_CM  = 30.0;         // [TINH CHỈNH] gần hơn mức này thì dừng (cm). Xe dừng không kịp: tăng.
const float OBSTACLE_CLEAR_CM = 50.0;         // [TINH CHỈNH] xa hơn mức này thì chạy tiếp (cm). Phải > OBSTACLE_DIST_CM.
const int   OBSTACLE_CONFIRM  = 1;            // [TINH CHỈNH] số lần đo liên tiếp để xác nhận có vật. Dừng nhầm: tăng.
const unsigned long OBSTACLE_MAX_WAIT_MS = 5000;     // [TINH CHỈNH] chờ vật cản tối đa (ms) trước khi báo lỗi.
const unsigned long ULTRASONIC_INTERVAL_MS = 50;     // [TINH CHỈNH] chu kỳ đo (ms). Không nên nhỏ hơn 50.
const unsigned long ULTRASONIC_TIMEOUT_US  = 4000;   // [TINH CHỈNH] thời gian chờ echo (us); 6000 ~ đo tối đa ~1 m.
                                                     // Lưu ý: pulseIn chặn tối đa thời gian này mỗi lần đo.

// ---- Mất line ----
const unsigned long LOST_COAST_MS = 120;      // [TINH CHỈNH] mất line thì chạy tiếp theo đà bao lâu (ms).
const unsigned long LOST_LINE_MS  = 4000;     // [TINH CHỈNH] mất line quá lâu -> lỗi LINE_LOST (ms).

// ---- An toàn ----
const unsigned long TRIP_TIMEOUT_MS = 90000;  // [TINH CHỈNH] một chuyến tối đa bao lâu (ms) -> lỗi TIMEOUT.
const unsigned long ARRIVED_HOLD_MS = 3000;   // [TINH CHỈNH] giữ trạng thái "đã tới đích" trước khi về sẵn sàng (ms).
// (STOP_ON_NO_CONTACT và HEARTBEAT_TIMEOUT_MS đã chuyển lên đầu file)
// =====================================================================

enum Mode {
  M_IDLE, M_FOLLOW, M_NUDGE, M_SETTLE, M_TURN, M_STRAIGHT, M_TO_POINT,
  M_UNLOAD_PUSH, M_UNLOAD_HOLD, M_UNLOAD_BACK, M_HOME, M_ARRIVED, M_ERROR
};
// Tên giai đoạn gửi cho dashboard (phải khớp thứ tự enum ở trên)
const char* MODE_NAMES[] = {
  "IDLE", "FOLLOW", "NUDGE", "SETTLE", "TURN", "STRAIGHT", "TO_POINT",
  "UNLOAD_PUSH", "UNLOAD_HOLD", "UNLOAD_BACK", "HOME", "ARRIVED", "ERROR"
};

Mode mode = M_IDLE;
char dest = '-';
const char* reason = "";
bool returning = false;
int  cornerDir = 0;
int  corners = 0;
unsigned long modeSince = 0, tripStart = 0, lastContact = 0, lastWifiCheck = 0, lastIpPrint = 0;
unsigned long kickUntil = 0;

// ---- PD ----
float pidLastError = 0;
unsigned long pidLastTime = 0, lineSeenAt = 0;
bool lostActive = false;

// ---- Siêu âm / vật cản ----
unsigned long lastUltrasonicTime = 0, obstaclePausedAt = 0;
float currentDistance = 150.0;
int   obstacleHits = 0;
bool  obstacleActive = false, wasObstaclePaused = false;

// ---- Lọc cảm biến ----
bool sStable[5], sCand[5];
unsigned long sCandSince[5];
unsigned long markSince = 0;

Servo cargoServo;
WebServer server(80);

// Thời gian đã trôi qua kể từ mốc t; nếu mốc nằm SAU now thì trả 0 (tránh tràn số không dấu).
static inline unsigned long since(unsigned long now, unsigned long t) {
  return ((long)(now - t) < 0) ? 0UL : (now - t);
}

static inline void startKick(unsigned long now) { kickUntil = now + KICK_MS; }

// ---- Nhật ký sự kiện gửi lên web (mã ASCII, server dịch sang tiếng Việt) ----
// Mã: GO:A  SEEN:L:1  TURNED:1  DROP:A  DONE:A  HOME
const int EV_MAX = 8;           // Số sự kiện lưu lại. Server hỏi mỗi 1 giây nên 8 là đủ.
char evMsg[EV_MAX][24];
unsigned long evSeq = 0;

void addEvent(const char* fmt, ...) {
  evSeq++;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(evMsg[evSeq % EV_MAX], sizeof(evMsg[0]), fmt, ap);
  va_end(ap);
}

// =====================================================================
//  ĐỘNG CƠ
// =====================================================================
void setSide(int a, int b, int en, int v) {
  digitalWrite(a, v > 0 ? HIGH : LOW);
  digitalWrite(b, v < 0 ? HIGH : LOW);
  analogWrite(en, map(abs(v), 0, 100, 0, 255));
}

void drive(int left, int right) {
  if (SWAP_MOTORS_LR) { int t = left; left = right; right = t; }
  left  = constrain(left  * LEFT_MOTOR_GAIN  / 100, -100, 100);
  right = constrain(right * RIGHT_MOTOR_GAIN / 100, -100, 100);
  setSide(IN1, IN2, ENA, left);
  setSide(IN3, IN4, ENB, right);
}

void setMode(Mode m) { mode = m; modeSince = millis(); markSince = 0; }

const char* stateName() {
  switch (mode) {
    case M_IDLE: return "IDLE";
    case M_UNLOAD_PUSH: case M_UNLOAD_HOLD: case M_UNLOAD_BACK: return "UNLOADING";
    case M_HOME: return "RETURNING";
    case M_ARRIVED: return "ARRIVED";
    case M_ERROR: return "ERROR";
    default: return returning ? "RETURNING" : "MOVING";
  }
}

// =====================================================================
//  CẢM BIẾN DÒ LINE
// =====================================================================
void initSensors() {
  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    sStable[i] = sCand[i] = (digitalRead(SENSOR_PINS[i]) == HIGH);
    sCandSince[i] = 0;
  }
}

void updateSensors(unsigned long now) {
  for (int i = 0; i < 5; i++) {
    bool raw = (digitalRead(SENSOR_PINS[i]) == HIGH);
    if (raw == sStable[i]) {
      sCand[i] = raw;
    } else if (raw != sCand[i]) {
      sCand[i] = raw;
      sCandSince[i] = now;
    } else if (since(now, sCandSince[i]) >= SENSOR_DEBOUNCE_MS) {
      sStable[i] = raw;
    }
  }
}

int readSensors(bool b[5]) {
  int n = 0;
  for (int i = 0; i < 5; i++) {
    b[i] = (sStable[i] == BLACK_IS_HIGH);
    if (b[i]) n++;
  }
  return n;
}

float lineError(const bool b[5], int n) {
  int sum = 0;
  for (int i = 0; i < 5; i++) if (b[i]) sum += SENSOR_WEIGHT[i];
  return (float)sum / n;
}

bool markConfirmed(bool cond, unsigned long now, unsigned long confirmMs) {
  if (!cond) { markSince = 0; return false; }
  if (markSince == 0) markSince = now;
  return (since(now, markSince) >= confirmMs);
}

// -1 = góc TRÁI, +1 = góc PHẢI, 0 = không phải góc
// [TINH CHỈNH] điều kiện nhận góc: 2 cảm biến một bên đen, 2 cảm biến bên kia trắng.
int cornerSide(const bool b[5]) {
  if (b[0] && b[1] && !b[3] && !b[4]) return -1;
  if (b[4] && b[3] && !b[0] && !b[1]) return +1;
  return 0;
}

// =====================================================================
//  BÁM LINE PD
// =====================================================================
void resetPID(unsigned long now) {
  pidLastError = 0;
  pidLastTime = 0;
  lostActive = false;
  lineSeenAt = now;
}

bool followLine(const bool b[5], int n, unsigned long now) {
  if (n > 0) {
    lineSeenAt = now;
    float error = lineError(b, n);
    if (lostActive) { pidLastError = error; lostActive = false; }
    if (since(now, pidLastTime) < PID_INTERVAL_MS) return true;
    pidLastTime = now;

    float correction = KP * error + KD * (error - pidLastError);
    pidLastError = error;

    int base = BASE_SPEED - (int)(SLOW_ON_CURVE * fabs(error));
    if (base < BASE_MIN_SPEED) base = BASE_MIN_SPEED;

    int maxSp = MAX_SPEED;
    if (now < kickUntil) { base = KICK_SPEED; maxSp = KICK_SPEED; }  // đang trong cú đẩy

    drive(constrain(base + (int)correction, MIN_SPEED, maxSp),
          constrain(base - (int)correction, MIN_SPEED, maxSp));
    return true;
  }

  unsigned long lost = since(now, lineSeenAt);
  if (lost > LOST_LINE_MS) return false;
  if (lost < LOST_COAST_MS) return true;
  lostActive = true;
  drive(0, 0);
  return true;
}

// =====================================================================
//  SIÊU ÂM
// =====================================================================
void initUltrasonic() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

float pingOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (d == 0) return -1;
  return (d * 0.0343) / 2.0;
}

void readUltrasonic(unsigned long now) {
  if (!OBSTACLE_ENABLED) return;
  if (since(now, lastUltrasonicTime) < ULTRASONIC_INTERVAL_MS) return;
  lastUltrasonicTime = now;

  float m = pingOnce();
  // Lọc mượt: 0.7 = tin số đo mới bao nhiêu. [TINH CHỈNH] tăng = nhạy hơn, giảm = ổn định hơn.
  if (m > 0) currentDistance = (0.7 * m) + (0.3 * currentDistance);
  else       currentDistance = 150.0;   // không có echo = coi như không có vật

  if (currentDistance < OBSTACLE_DIST_CM) { if (obstacleHits < 100) obstacleHits++; }
  else obstacleHits = 0;

  if (!obstacleActive && obstacleHits >= OBSTACLE_CONFIRM) obstacleActive = true;
  else if (obstacleActive && currentDistance > OBSTACLE_CLEAR_CM) obstacleActive = false;
}

// =====================================================================
//  CHUYẾN ĐI
// =====================================================================
void fail(const char* why) {
  drive(0, 0);
  cargoServo.write(SERVO_REST_ANGLE);
  kickUntil = 0;
  reason = why;
  Serial.print("ERROR: ");
  Serial.println(why);
  setMode(M_ERROR);
}

void arrive() {
  drive(0, 0);
  addEvent("HOME");
  setMode(M_ARRIVED);
}

void beginTrip(char d, unsigned long now) {
  dest = d;
  reason = "";
  corners = 0;
  returning = false;
  tripStart = now;
  wasObstaclePaused = false;
  resetPID(now);
  startKick(now);                     // cú đẩy lúc xuất phát
  setMode(M_FOLLOW);
  drive(KICK_SPEED, KICK_SPEED);
  addEvent("GO:%c", d);
}

void runTrip(unsigned long now) {
  if (mode == M_IDLE || mode == M_ARRIVED || mode == M_ERROR) return;

  bool b[5];
  int n = readSensors(b);

  if (STOP_ON_NO_CONTACT && since(now, lastContact) > HEARTBEAT_TIMEOUT_MS) { fail("NO_CONTACT"); return; }
  if (since(now, tripStart) > TRIP_TIMEOUT_MS)        { fail("TIMEOUT"); return; }

  // ---- Vật cản: chỉ kiểm tra khi đang chạy bám line ----
  bool movingMode = (mode == M_FOLLOW || mode == M_STRAIGHT || mode == M_TO_POINT || mode == M_HOME);
  if (OBSTACLE_ENABLED && movingMode && obstacleActive) {
    if (!wasObstaclePaused) {
      drive(0, 0);
      wasObstaclePaused = true;
      obstaclePausedAt = now;
    } else if (since(now, obstaclePausedAt) > OBSTACLE_MAX_WAIT_MS) {
      fail("OBSTACLE");
    }
    return;
  }
  if (wasObstaclePaused && !obstacleActive) {
    wasObstaclePaused = false;
    modeSince += since(now, obstaclePausedAt);  // không tính thời gian chờ vào các mốc thời gian của chặng
    resetPID(now);
    startKick(now);                             // vừa đứng yên, đẩy lại cho có đà
    markSince = 0;
  }

  switch (mode) {
    // ---- Bám line, dò góc ----
    case M_FOLLOW: {
      bool ignoring = since(now, modeSince) < CORNER_IGNORE_MS;
      int side = ignoring ? 0 : cornerSide(b);
      if (markConfirmed(side != 0, now, CORNER_CONFIRM_MS)) {
        cornerDir = side;
        addEvent("SEEN:%c:%d", side < 0 ? 'L' : 'R', corners + 1);
        drive(NUDGE_SPEED, NUDGE_SPEED);
        setMode(M_NUDGE);
      } else if (!followLine(b, n, now)) {
        fail("LINE_LOST");
      }
      break;
    }

    case M_NUDGE:
      if (since(now, modeSince) >= CORNER_NUDGE_MS) { drive(0, 0); setMode(M_SETTLE); }
      break;

    case M_SETTLE:
      if (since(now, modeSince) >= CORNER_SETTLE_MS) setMode(M_TURN);
      break;

    // ---- Xoay tại chỗ ----
    case M_TURN: {
      unsigned long te = since(now, modeSince);
      int spd = (te < TURN_BLIND_MS) ? TURN_SPEED : TURN_SEEK_SPEED;
      if (cornerDir < 0) drive(-spd, spd);
      else               drive(spd, -spd);

      bool caught = (te > TURN_BLIND_MS) && markConfirmed(b[2], now, TURN_CATCH_CONFIRM_MS);
      if (caught) {
        corners++;
        addEvent("TURNED:%d", corners);
        setMode(M_STRAIGHT);          // rẽ xong thì đi thẳng STRAIGHT_MS rồi mới bám line
      } else if (te > TURN_TIMEOUT_MS) {
        fail("TURN_FAIL");
      }
      break;
    }

    // ---- Đi thẳng sau khi rẽ xong (không bám line) ----
    case M_STRAIGHT:
      drive(STRAIGHT_SPEED, STRAIGHT_SPEED);
      if (since(now, modeSince) >= STRAIGHT_MS) {
        resetPID(now);
        startKick(now);               // cú đẩy sau khi đi thẳng xong
        if (!returning) {
          int target = (dest == 'A') ? POINT_A_CORNER : POINT_B_CORNER;
          setMode(corners >= target ? M_TO_POINT : M_FOLLOW);
        } else {
          setMode(corners >= SQUARE_CORNERS ? M_HOME : M_FOLLOW);
        }
      }
      break;

    // ---- Bám line tới vạch ngang của điểm A/B rồi dừng dỡ hàng ----
    case M_TO_POINT: {
      unsigned long t = since(now, modeSince);
      bool wide = (n >= POINT_MARK_COUNT) && b[0] && b[4];
      if (t > POINT_IGNORE_MS && markConfirmed(wide, now, POINT_CONFIRM_MS)) {
        drive(0, 0);
        addEvent("DROP:%c", dest);
        cargoServo.write(SERVO_PUSH_ANGLE);
        setMode(M_UNLOAD_PUSH);
      } else if (t > POINT_MAX_MS) {
        fail("POINT_MISSED");         // không thấy vạch ngang của điểm
      } else if (!followLine(b, n, now)) {
        fail("LINE_LOST");
      }
      break;
    }

    // ---- Servo dỡ hàng ----
    case M_UNLOAD_PUSH:
      drive(0, 0);
      if (since(now, modeSince) >= SERVO_PUSH_MS) setMode(M_UNLOAD_HOLD);
      break;

    case M_UNLOAD_HOLD:
      drive(0, 0);
      if (since(now, modeSince) >= SERVO_HOLD_MS) {
        cargoServo.write(SERVO_REST_ANGLE);
        setMode(M_UNLOAD_BACK);
      }
      break;

    case M_UNLOAD_BACK:
      drive(0, 0);
      if (since(now, modeSince) >= SERVO_RETURN_MS) {
        addEvent("DONE:%c", dest);
        returning = true;
        resetPID(now);
        startKick(now);               // cú đẩy sau khi đổ hàng xong
        setMode(M_FOLLOW);
      }
      break;

    // ---- Đoạn cuối: bám line tới vạch đích ----
    case M_HOME: {
      unsigned long t = since(now, modeSince);
      bool seen;
      if (HOME_USE_MARK) {
        bool wide = (n >= HOME_MARK_COUNT) && b[0] && b[4];
        seen = (t > HOME_IGNORE_MS) && markConfirmed(wide, now, HOME_CONFIRM_MS);
      } else {
        seen = (t >= HOME_RUN_MS);
      }
      if (seen) {
        arrive();
      } else if (HOME_USE_MARK && t > HOME_MAX_MS) {
        fail("HOME_MISSED");
      } else if (!followLine(b, n, now)) {
        fail("LINE_LOST");
      }
      break;
    }

    default: break;
  }
}

// =====================================================================
//  HTTP ENDPOINTS
// =====================================================================
void sendStatus(int code = 200, const char* err = nullptr) {
  String j;
  j.reserve(400);
  j += "{\"ok\":";
  j += err ? "false" : "true";
  j += ",\"state\":\"";
  j += stateName();
  j += "\",\"dest\":\"";
  j += dest;
  j += "\"";
  if (mode != M_IDLE && mode != M_ARRIVED && mode != M_ERROR) {
    j += ",\"phase\":\"";
    j += MODE_NAMES[mode];
    j += "\",\"corners\":";
    j += String(corners);
  }
  if (mode == M_ERROR) {
    j += ",\"reason\":\"";
    j += reason;
    j += "\"";
  }

  j += ",\"distance\":";
  j += String(currentDistance, 1);
  j += ",\"obstacle\":";
  j += obstacleActive ? "true" : "false";

  // Sự kiện gần nhất (tối đa EV_MAX), mỗi sự kiện có số thứ tự id tăng dần
  j += ",\"seq\":";
  j += String(evSeq);
  j += ",\"events\":[";
  unsigned long firstId = (evSeq > EV_MAX ? evSeq - EV_MAX + 1 : 1);
  for (unsigned long id = firstId; id <= evSeq; id++) {
    if (id > firstId) j += ",";
    j += "{\"id\":";
    j += String(id);
    j += ",\"m\":\"";
    j += evMsg[id % EV_MAX];
    j += "\"}";
  }
  j += "]";

  if (err) {
    j += ",\"err\":\"";
    j += err;
    j += "\"";
  }
  j += "}";
  server.send(code, "application/json", j);
}

void onStatus() { lastContact = millis(); sendStatus(); }

void onGo() {
  lastContact = millis();
  String d = server.arg("dest");
  if (d != "A" && d != "B") { sendStatus(400, "BAD_DEST"); return; }
  if (mode != M_IDLE)       { sendStatus(409, "BUSY");     return; }
  bool b[5];
  if (readSensors(b) == 0)  { sendStatus(409, "NOT_ON_LINE"); return; }
  beginTrip(d[0], millis());
  sendStatus();
}

void onStop() {
  lastContact = millis();
  drive(0, 0);
  cargoServo.write(SERVO_REST_ANGLE);
  kickUntil = 0;
  dest = '-';
  reason = "";
  returning = false;
  wasObstaclePaused = false;
  setMode(M_IDLE);
  sendStatus();
}

// =====================================================================
//  SETUP & LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  initSensors();
  initUltrasonic();
  drive(0, 0);

  ESP32PWM::allocateTimer(2);
  cargoServo.setPeriodHertz(50);
  cargoServo.attach(PIN_SERVO, 500, 2400);  // [TINH CHỈNH] 500-2400 us: đổi nếu servo không quay đủ góc.
  cargoServo.write(SERVO_REST_ANGLE);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && since(millis(), t0) < WIFI_BOOT_WAIT_MS) delay(500);

  // IP của xe: điền vào run.bat / run.ps1 (ESP32_URL)
  Serial.print("IP cua xe: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("chua noi duoc WiFi"));

  server.on("/status", onStatus);
  server.on("/go", onGo);
  server.on("/stop", onStop);
  server.begin();

  lastContact = millis();
}

void loop() {
  // QUAN TRỌNG: xử lý HTTP TRƯỚC, rồi mới đọc millis() (tránh lỗi NO_CONTACT do tràn số).
  server.handleClient();
  unsigned long now = millis();

  updateSensors(now);
  readUltrasonic(now);

  if (WiFi.status() != WL_CONNECTED && since(now, lastWifiCheck) > WIFI_RECHECK_MS) {
    lastWifiCheck = now;
    WiFi.reconnect();
  }

  // In địa chỉ IP mỗi IP_PRINT_MS (ms) ra Serial Monitor để dễ lấy điền vào run.bat / run.ps1
  if (since(now, lastIpPrint) >= IP_PRINT_MS) {
    lastIpPrint = now;
    Serial.print("IP cua xe: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("chua noi duoc WiFi"));
  }

  runTrip(now);

  if (mode == M_ARRIVED && since(now, modeSince) > ARRIVED_HOLD_MS) {
    dest = '-';
    returning = false;
    setMode(M_IDLE);
  }
}
