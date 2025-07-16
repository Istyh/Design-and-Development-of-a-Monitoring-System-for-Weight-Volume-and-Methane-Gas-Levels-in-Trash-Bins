#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <math.h>

#define WIFI_SSID "uxshin"
#define WIFI_PASSWORD "xixixixi"

#define API_KEY "AIzaSyBMsqV-LSOI0kdsPjPrIW0WU1SO06wHHoU"
#define DATABASE_URL "monitoring-tempat-sampah-52c13-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Sensor pin
#define TRIG_PIN 26
#define ECHO_PIN 25
#define HX711_DOUT 18
#define HX711_SCK 19
#define MQ2_PIN 35

// Load Cell
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float offset = 0.0;
unsigned long startTime;

// MQ-2
const float RL = 10.0; // disesuaikan
const float VCC = 3.3;
float Ro = 10.0;
bool sudahKalibrasi = false;
const int baselineSamples = 50;
const float cleanAirRatio = 9.83;

// Tempat sampah
const int tinggi_tempat_sampah_cm = 22;
const float diameter_cm = 24.0;
const float jari2 = diameter_cm / 2.0;
const float volume_max_cm3 = 3.14 * jari2 * jari2 * tinggi_tempat_sampah_cm;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000;

int bacaUltrasonik();
void kalibrasiMQ2();
float bacaPPM_MQ2();

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);

  LoadCell.begin();
  float calibrationValue = 696.0;
  EEPROM.begin(512);
  EEPROM.get(calVal_eepromAdress, calibrationValue);

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Load cell startup complete");
  }
  delay(100);
  offset = LoadCell.getData();
  startTime = millis();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase sign-up succeeded");
  } else {
    Serial.printf("Firebase sign-up failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  analogReadResolution(12);
  kalibrasiMQ2();
}

void loop() {
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0;

    if (LoadCell.update()) newDataReady = true;

    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float berat = LoadCell.getData() - offset;
        if (berat < 0) berat = 0;

        // === Sensor Ultrasonik ===
        int jarak = bacaUltrasonik();
        int tinggi_sampah = tinggi_tempat_sampah_cm - jarak;
        if (tinggi_sampah < 0) tinggi_sampah = 0;
        if (tinggi_sampah > tinggi_tempat_sampah_cm) tinggi_sampah = tinggi_tempat_sampah_cm;
        float volume_sampah_cm3 = 3.14 * jari2 * jari2 * tinggi_sampah;
        float persentase_volume = (volume_sampah_cm3 / volume_max_cm3) * 100.0;

        // === MQ2 ===
        float ppm = bacaPPM_MQ2();

        // === Kirim ke Firebase ===
        Firebase.RTDB.setInt(&fbdo, "/TS1/KETINGGIAN_SAMPAH_CM", tinggi_sampah);
        Firebase.RTDB.setFloat(&fbdo, "/TS1/VOLUME_PERSEN", persentase_volume);
        Firebase.RTDB.setFloat(&fbdo, "/TS1/LOAD_CELL_WEIGHT", berat);
        Firebase.RTDB.setFloat(&fbdo, "/TS1/MQ2_PPM", ppm);

        String status;
        if (persentase_volume <= 50.0) {
           status = "Not Full";
        } else if (persentase_volume <= 80.0) {
           status = "Warning";
        } else {
           status = "Full";
        }
        Firebase.RTDB.setString(&fbdo, "/TS1/TRASH_STATUS", status);


        // === Output Serial Debug Baru ===
        Serial.println("Ketinggian: " + String(tinggi_sampah) + " cm");
        Serial.println("Volume: " + String(persentase_volume) + " %");
        Serial.println("Berat: " + String(berat) + " g");
        Serial.println("MQ-2 PPM: " + String(ppm));
        Serial.println("=========================");

        newDataReady = 0;
        t = millis();
      }
    }
  }

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell.tareNoDelay();
      delay(500);
      offset = LoadCell.getData();
    }
  }

  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

int bacaUltrasonik() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  return distance;
}

void kalibrasiMQ2() {
  Serial.println("Kalibrasi MQ-2 dimulai. Pastikan lingkungan bebas gas.");
  delay(5000);
  float rs_sum = 0.0;
  for (int i = 0; i < baselineSamples; i++) {
    int adc = analogRead(MQ2_PIN);
    float Vout = ((float)adc * VCC) / 4095.0;
    if (Vout == 0) Vout = 0.01;
    float Rs = (VCC - Vout) * RL / Vout;
    rs_sum += Rs;
    delay(100);
  }
  float rs_avg = rs_sum / baselineSamples;
  Ro = rs_avg / 9.8;
  sudahKalibrasi = true;
  Serial.print("Kalibrasi selesai. Ro = ");
  Serial.println(Ro, 3);
}

float bacaPPM_MQ2() {
  if (!sudahKalibrasi) return 0.0;
  int adc = analogRead(MQ2_PIN);
  float Vout = ((float)adc * VCC) / 4095.0;
  if (Vout == 0) Vout = 0.01;
  float Rs = (VCC - Vout) * RL / Vout;
  float ratio = Rs / Ro;
  float ppm = 1000 * pow(ratio, -2.2);
  if (ppm < 0) ppm = 0;
  return ppm;
}
