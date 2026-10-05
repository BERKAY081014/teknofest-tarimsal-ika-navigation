/**
 * ============================================================================
 * Proje: TEKNOFEST Tarımsal İnsansız Kara Aracı (TİKA) Seyrüsefer Yazılımı
 * Rol: Takım Kaptanı & Gömülü Yazılım Mimarı (Berkay Bilgin)
 * Platform: Arduino Mega 2560 / STM32 Cortex-M
 * Sensörler: NEO-6M GPS, HMC5883L Dijital Pusula, 3x HC-SR04 Ultrasonik
 * Sürücü: Cytron / BTS7960 43A H-Bridge Çift DC Motor Sürücü
 * ============================================================================
 */

#include <Wire.h>
#include <TinyGPS++.h>

// ---------- Pin Tanımları ----------
#define PIN_MOTOR_LEFT_PWM   5
#define PIN_MOTOR_LEFT_DIR   6
#define PIN_MOTOR_RIGHT_PWM  9
#define PIN_MOTOR_RIGHT_DIR  10

// Ultrasonik Sensör Pinleri (Sol, Ön, Sağ)
#define PIN_TRIG_FRONT 22
#define PIN_ECHO_FRONT 23
#define PIN_TRIG_LEFT  24
#define PIN_ECHO_LEFT  25
#define PIN_TRIG_RIGHT 26
#define PIN_ECHO_RIGHT 27

// GPS Seri Haberleşme
#define GPSSerial Serial1
TinyGPSPlus gps;

// Hedef Waypoint Koordinatları (Enlem, Boylam)
struct Waypoint {
  double latitude;
  double longitude;
};

const Waypoint WAYPOINTS[] = {
  {40.352410, 27.973120}, // 1. Tarla Başlangıç Noktası
  {40.352850, 27.973650}, // 2. Ara Sıra Noktası
  {40.353200, 27.974100}  // 3. Hasat Tahliye Noktası
};
const int TOTAL_WAYPOINTS = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
int currentWaypointIndex = 0;

// Yön ve Navigasyon Parametreleri
float targetHeading = 0.0f;
float currentHeading = 0.0f;
float headingError = 0.0f;
const float BASE_SPEED = 160.0f; // 0 - 255 PWM
const float OBSTACLE_DISTANCE_LIMIT = 35.0f; // cm cinsinden güvenlik mesafesi

// Fonksiyon Prototipleri
float readDistanceCM(int trigPin, int echoPin);
float readCompassHeading();
double calculateDistanceToWaypoint(double lat1, double lon1, double lat2, double lon2);
double calculateBearingToWaypoint(double lat1, double lon1, double lat2, double lon2);
void driveMotors(int leftSpeed, int rightSpeed);
void executeObstacleAvoidance();

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600);
  Wire.begin();

  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);

  pinMode(PIN_TRIG_FRONT, OUTPUT);
  pinMode(PIN_ECHO_FRONT, INPUT);
  pinMode(PIN_TRIG_LEFT, OUTPUT);
  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);

  driveMotors(0, 0);
  Serial.println("[TİKA] TEKNOFEST Otonom Tarımsal İKA Başlatıldı.");
}

void loop() {
  // 1. GPS Akışını Oku
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  // 2. Engel Algılama (Güvenlik Önceliği)
  float distFront = readDistanceCM(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
  if (distFront > 0.0f && distFront < OBSTACLE_DISTANCE_LIMIT) {
    Serial.println("[DİKKAT] Önde engel tespit edildi! Manevra başlatılıyor...");
    executeObstacleAvoidance();
    return;
  }

  // 3. Hedefe Ulaşım Kontrolü
  if (gps.location.isValid()) {
    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();
    double targetLat  = WAYPOINTS[currentWaypointIndex].latitude;
    double targetLon  = WAYPOINTS[currentWaypointIndex].longitude;

    double distToTarget = calculateDistanceToWaypoint(currentLat, currentLon, targetLat, targetLon);

    if (distToTarget < 2.5) { // 2.5 metre waypoint toleransı
      Serial.printf("[WAYPOINT] Hedef #%d noktasina ulasildi!\n", currentWaypointIndex + 1);
      currentWaypointIndex++;
      if (currentWaypointIndex >= TOTAL_WAYPOINTS) {
        Serial.println("[GÖREV TAMAMLANDI] Tüm rota basariyla tarandi. Arac durduruluyor.");
        driveMotors(0, 0);
        while (1) { delay(1000); }
      }
    }

    // 4. Açısal Yönelim ve Diferansiyel Dümenleme (P Kontrol)
    targetHeading = (float)calculateBearingToWaypoint(currentLat, currentLon, targetLat, targetLon);
    currentHeading = readCompassHeading();

    headingError = targetHeading - currentHeading;
    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;

    // Oransal Dönüş Katsayısı
    float turnAdjustment = headingError * 1.8f;
    int leftMotorSpeed  = (int)constrain(BASE_SPEED + turnAdjustment, 0.0f, 240.0f);
    int rightMotorSpeed = (int)constrain(BASE_SPEED - turnAdjustment, 0.0f, 240.0f);

    driveMotors(leftMotorSpeed, rightMotorSpeed);
  } else {
    // GPS kilitlenmesi bekleniyor
    driveMotors(0, 0);
    delay(200);
  }
}

void driveMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(PIN_MOTOR_LEFT_DIR, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, abs(leftSpeed));
  digitalWrite(PIN_MOTOR_RIGHT_DIR, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, abs(rightSpeed));
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout
  if (duration == 0) return 999.0f;
  return (float)duration * 0.0343f / 2.0f;
}

float readCompassHeading() {
  // HMC5883L I2C Manyetometre Açı Okuma
  Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x1E, 6);
  if (Wire.available() >= 6) {
    int16_t x = Wire.read() << 8 | Wire.read();
    int16_t z = Wire.read() << 8 | Wire.read();
    int16_t y = Wire.read() << 8 | Wire.read();
    float heading = atan2((float)y, (float)x) * 180.0f / 3.14159f;
    if (heading < 0) heading += 360.0f;
    return heading;
  }
  return 0.0f;
}

double calculateDistanceToWaypoint(double lat1, double lon1, double lat2, double lon2) {
  // Haversine Formülü (Metre cinsinden mesafe)
  double dLat = (lat2 - lat1) * 3.14159 / 180.0;
  double dLon = (lon2 - lon1) * 3.14159 / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * 3.14159 / 180.0) * cos(lat2 * 3.14159 / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000.0 * c;
}

double calculateBearingToWaypoint(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * 3.14159 / 180.0;
  double y = sin(dLon) * cos(lat2 * 3.14159 / 180.0);
  double x = cos(lat1 * 3.14159 / 180.0) * sin(lat2 * 3.14159 / 180.0) -
             sin(lat1 * 3.14159 / 180.0) * cos(lat2 * 3.14159 / 180.0) * cos(dLon);
  double bearing = atan2(y, x) * 180.0 / 3.14159;
  if (bearing < 0) bearing += 360.0;
  return bearing;
}

void executeObstacleAvoidance() {
  driveMotors(-100, -100); // Geri çekil
  delay(500);
  driveMotors(150, -150);  // Sağa keskin dön
  delay(600);
}\n