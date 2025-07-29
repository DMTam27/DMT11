#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "time.h" 
#include <DHT.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include "Audio.h"
#include "esp_sleep.h"

// WiFi
const char* ssid = "**********";
const char* password = "***********";
// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==== SD Card Pins ====
#define SD_CS     5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK  18

// ==== I2S Audio (MAX98357A) ====
#define I2S_DOUT 25
#define I2S_BCLK 15
#define I2S_LRC   4

Audio audio;

// ==== Nút ADC ====
const int nutADC = 34;
// Ngưỡng ADC cho từng nút
#define NGUONG_NHAC 1500
#define NGUONG_CUTE 800
#define BAI_DAU 9
#define BAI_CUOI 30
#define SO_BAI 22
#define THOI_GIAN_NHAN_GIU 1000   // Nhấn giữ > 1s để tắt nhạc
#define KHOANG_CACH_NHAN_DUP 500     // 2 lần nhấn trong 0.5s → đúp
#define DEBOUNCE_TIME 300            // Chống rung nút


// Trạng thái toggle
bool trangThaiNhac = false;
bool nutDangDuocNhan = false;
unsigned long thoiGianNhat = 0;
unsigned long thoiGianTha = 0;
unsigned long lastPressTime = 0;
bool trangThaiCute = false;
unsigned long cuteBatLuc = 0;
// Biến thời gian debounce
unsigned long lastDebounceTime = 0;


int soBaiHienTai = BAI_DAU;
//======= Hàm phát nhạc =======
void playSong(int soBai) {
  char filename[20];
  snprintf(filename, sizeof(filename), "/%05d.wav", soBai);
  Serial.print("▶️ Phat: ");
  Serial.println(filename);
  audio.stopSong();
  audio.connecttoFS(SD, filename);
}


// ==== DHT11 ====
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
// ==== Biến toàn cục dùng chung giữa các task ====
volatile float globalTemp = 0.0;
volatile float globalHumi = 0.0;

// ==== HC-SR04 ====
#define TRIG_PIN 33
#define ECHO_PIN 32
// ==== SERVO ====
#define SERVO_PIN 26
Servo myServo;

// ==== RUNG ====
#define RUNG_PIN 27

// ==== HỒNG NGOẠI, LED ĐỎ, CÒI ====
#define IR_SENSOR_PIN 17
#define LED_DO_PIN    14
#define COI_PIN       16
bool vatTruocDo = false;

// ==== Biến trạng thái ====
bool servoOpened = false;

// OpenWeatherMap
const char* weatherURL = "http://api.openweathermap.org/data/2.5/forecast?q=Hanoi&appid=3fa2915dc8b35d4aa24e22ce2bc77fdf&units=metric";

// NTP
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;
const char* thuTrongTuan[] = {
  "CN", "T2", "T3", "T4", "T5", "T6", "T7"
};

int gioHienTai = 0;
int phutHienTai = 0;
        // ==== LỊCH PHÁT NHẠC THEO GIỜ ====
// Cấu trúc lưu thông tin giờ, phút, tên file và cờ đã phát
struct GioNhac {
  int gio;
  int phut;
  int soBai;     // số bài thay vì tên file
  bool daPhat;
};

GioNhac lichPhat[] = {
  {5, 30, 4, false},    // Gọi chị Béo dậy 
 {5, 31, 4, false},
 {5, 32, 4, false},
 {5, 43, 4, false},
 {5, 55, 4, false},
  {6, 0, 4, false},
  {7, 30, 5, false},    // Nhắc chị Thảo đi học sáng 
   {7,32, 5, false},
  {7, 34, 5, false},
  {7, 36, 5, false},
  {7, 38, 5, false},
  {7, 40, 5, false},
  {8, 0, 7, false},
  {9, 0, 7, false},     // Hỏi bà đang làm gì đấy 
  {10, 30, 6, false},   // Hỏi bà nấu cơm chưa 
  {17, 0, 8, false},    // Nhắc chị Béo tập thể dục 
  {17, 15, 8, false},
  {17, 40, 6, false},
  {18, 0, 1, false}, // Chào bố 
  {18, 5, 1, false},
  {18, 10, 1, false},
  {18, 15, 1, false},
  {18, 25, 1, false},
  {18, 26, 1, false},
  {18, 28, 1, false},
  {18, 30, 1, false},   // Chào bố 
  {19, 28, 3, false},
  {19, 30, 3, false}, 
  {19, 31, 3, false}, 
  {19, 32, 3, false},   // Nhắc bà uống thuốc 
  
  {19, 50, 5, false}, 
  {19, 51, 5, false}, 
  {19, 52, 5, false},
  {19, 53, 5, false},
   {19, 54, 5, false},
    {19, 55, 5, false}, // Nhắc chị Thảo đi học tối
     
  {21, 28, 9, false},
  {21, 29, 9, false},
  {21, 30, 9, false},
  {21, 31, 9, false},
  {21, 32, 9, false}    // Chúc mọi người ngủ ngon 
};

const int soMoc = sizeof(lichPhat) / sizeof(lichPhat[0]);


// Dữ liệu thời tiết
struct WeatherInfo {
  float currentTemp;
  String currentDesc;
  float forecast3h;
  float forecast12h;
  String iconCode;
};
QueueHandle_t weatherQueue;
// Icon thời tiết đơn giản
String iconToShortText(String icon) {
  if (icon.startsWith("01")) return "Nang";
  if (icon.startsWith("02")) return "May it";
  if (icon.startsWith("03")) return "May dai";
  if (icon.startsWith("04")) return "Nhieu may";
  if (icon.startsWith("09") || icon.startsWith("10")) return "Mua";
  if (icon.startsWith("11")) return "Dong";
  if (icon.startsWith("13")) return "Tuyet";
  if (icon.startsWith("50")) return "Suong";
  return "???";
}


//===== Task phat am thanh theo realtime ======
void taskPhatNhacTheoLich(void *parameter) {
  while (1) {
    for (int i = 0; i < soMoc; i++) {
      if (gioHienTai == lichPhat[i].gio &&
          phutHienTai == lichPhat[i].phut &&
          !lichPhat[i].daPhat) {
        Serial.printf("🎵 Phat bai %d luc %02d:%02d\n", lichPhat[i].soBai, lichPhat[i].gio, lichPhat[i].phut);
        playSong(lichPhat[i].soBai);
        lichPhat[i].daPhat = true;
      }

      // Reset lại nếu qua thời điểm
      if (!(gioHienTai == lichPhat[i].gio && phutHienTai == lichPhat[i].phut)) {
        lichPhat[i].daPhat = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // kiểm tra mỗi giây
  }
}


// === Task đọc nút ADC ===
void taskDocNutADC(void *parameter) {
  while (true) {
    int adcVal = analogRead(nutADC);
    unsigned long now = millis();

    bool nutNhacDuocNhan = (adcVal > 1300 && adcVal < 2500);

    if (nutNhacDuocNhan && !nutDangDuocNhan) {
      nutDangDuocNhan = true;
      thoiGianNhat = now;
    }

    if (!nutNhacDuocNhan && nutDangDuocNhan) {
      nutDangDuocNhan = false;
      thoiGianTha = now;
      unsigned long thoiGianNhan = thoiGianTha - thoiGianNhat;

      if (thoiGianNhan >= THOI_GIAN_NHAN_GIU) {
        // Nhấn giữ → tắt nhạc
        //Serial.println("⛔ Nhấn giữ → TẮT NHẠC");
        trangThaiNhac = false;
        audio.stopSong();
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Nhac da tat.");
        display.display();

      } else if (now - lastPressTime < KHOANG_CACH_NHAN_DUP) {
        // Nhấn đúp → random bài
       // Serial.println("🎲 Nhấn đúp → PHÁT RANDOM");
        int baiRandom;
        do {
          baiRandom = random(BAI_DAU, BAI_CUOI + 1);
        } while (baiRandom == soBaiHienTai);
        soBaiHienTai = baiRandom;
        trangThaiNhac = true;
        playSong(soBaiHienTai);

      } else {
        // Nhấn đơn → phát bài tiếp
        lastPressTime = now;
        soBaiHienTai++;
        if (soBaiHienTai > BAI_CUOI) soBaiHienTai = BAI_DAU;
        trangThaiNhac = true;
       // Serial.print("▶️ Nhấn → bài kế: ");
       // Serial.println(soBaiHienTai);
        playSong(soBaiHienTai);
      }
    }

    // Nút cute (ADC 400–1200)
    if (adcVal > 400 && adcVal < 1200 && (now - lastDebounceTime > DEBOUNCE_TIME)) {
      trangThaiCute = true;
      cuteBatLuc = millis();
     // Serial.println("🎀 CUTE: BẬT (hiển thị 10 giây)");
      lastDebounceTime = now;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


// Task lấy thời tiết mỗi 60 giây
void TaskWeatherUpdate(void* pvParameters) {
  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(weatherURL);
      int code = http.GET();

      if (code == 200) {
        String payload = http.getString();
        DynamicJsonDocument doc(4096);
        DeserializationError err = deserializeJson(doc, payload);

        if (!err) {
          WeatherInfo info;
          info.currentTemp = doc["list"][0]["main"]["temp"];
          info.currentDesc = doc["list"][0]["weather"][0]["description"].as<String>();
          info.iconCode = doc["list"][0]["weather"][0]["icon"].as<String>();
          info.forecast3h = doc["list"][1]["main"]["temp"];
          info.forecast12h = doc["list"][4]["main"]["temp"];
          xQueueOverwrite(weatherQueue, &info);
        }
      }

      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(60000)); // Cập nhật mỗi 60 giây
  }
}

void TaskDisplayUpdate(void* pvParameters) {
  struct tm timeinfo;
  WeatherInfo current = {0};

  while (1) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Nếu Cute đang bật → kiểm tra thời gian hiển thị
    if (trangThaiCute) {
      if (millis() - cuteBatLuc > 10000) {
        trangThaiCute = false;  // Tắt chế độ sau 10 giây
        continue;  // Quay lại loop
      }

      // 🧸 Hiển thị riêng 1 trang dự báo thời tiết
      WeatherInfo updated;
      if (xQueueReceive(weatherQueue, &updated, 0) == pdTRUE) {
        current = updated;
      }

      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("==DU BAO THOI TIET==");

      display.setCursor(0, 12);
      display.printf("Now: %.1fC %s", current.currentTemp, iconToShortText(current.iconCode).c_str());

      display.setCursor(0, 24);
      display.printf("Desc: %s", current.currentDesc.c_str());
    
      display.setTextSize(1);
      display.setCursor(0, 36);
      display.printf("3h: %.1fC", current.forecast3h);
      display.setCursor(0, 48);
      display.printf("12h: %.1fC", current.forecast12h);

      display.display();
      vTaskDelay(pdMS_TO_TICKS(1000));  // Cập nhật mỗi 1 giây khi ở chế độ cute
      continue;
    }

    // Mặc định: hiển thị giờ, nhiệt độ, độ ẩm
    if (getLocalTime(&timeinfo)) {

      // Cập nhật biến dùng chung
  gioHienTai = timeinfo.tm_hour;
  phutHienTai = timeinfo.tm_min;
  // === Kiểm tra nếu giờ >= 22:00 thì tính thời gian ngủ đến 5h sáng ===
if (gioHienTai >= 22  ) {
  int secondsToSleep = ((24 - gioHienTai + 5) * 3600) - (phutHienTai * 60);
  Serial.printf("😴 Ngủ đến 5h sáng... Tổng thời gian ngủ: %d giây\n", secondsToSleep);

  // Hiển thị OLED trước khi ngủ
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println("Di ngu toi 5h...");
  display.display();
  delay(10000); // Cho người dùng đọc trước khi ngủ
  display.clearDisplay();

  // Ngắt WiFi, chuẩn bị ngủ
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup((uint64_t)secondsToSleep * 1000000ULL);
  esp_deep_sleep_start();
}


      display.setTextSize(2);
      display.setCursor(0, 0);
      display.printf("%s- %02d/%02d", thuTrongTuan[timeinfo.tm_wday], timeinfo.tm_mday, timeinfo.tm_mon + 1);
      display.setCursor(0, 17);
      display.printf("   %02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    }

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.printf("Nhiet do: %.1fC", globalTemp);
    display.setCursor(0, 52);
    display.printf("Do am: %.0f%%", globalHumi);

    display.display();
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);  // Đợi ổn định nguồn

  // ===== Kết nối WiFi =====
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // ===== Khởi tạo cảm biến, chân IO =====
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RUNG_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LED_DO_PIN, OUTPUT);
  pinMode(COI_PIN, OUTPUT);

  // ===== Khởi tạo Servo =====
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(0);

  // ===== Khởi tạo OLED =====
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("⚠️ OLED not found!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("Loading...");
  display.display();
  delay(1000);

  // ===== SD Card =====
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD card error!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Loi the SD!");
    display.display();
    while (true);
  }

  // ===== Audio =====
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // Có thể tăng lên đến 21 nếu cần

  // ===== Tạo queue dự báo thời tiết =====
  weatherQueue = xQueueCreate(1, sizeof(WeatherInfo));

  // ===== Tạo các task song song =====
  xTaskCreatePinnedToCore(TaskWeatherUpdate, "Weather", 5096, NULL, 0, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplayUpdate, "Display", 5096, NULL, 1, NULL, 1);
  xTaskCreate(taskDocNutADC, "TaskDocNutADC", 6096, NULL, 1, NULL);
  xTaskCreatePinnedToCore(taskThungRac, "Task_ThungRac", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskPhatNhacTheoLich, "PhatNhacTheoLich",4096, NULL, 1, NULL, 1);


  Serial.println("🚀 Setup hoàn tất!");
}


// ======= Đọc khoảng cách HC-SR04 =========
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.0343 / 2;
  return distance;
}

// ======= TASK FreeRTOS =========
void taskThungRac(void *parameter) {
  while (1) {
    globalTemp = dht.readTemperature();
    globalHumi = dht.readHumidity();
    float distance = readDistanceCM();
    int rung = digitalRead(RUNG_PIN);
    int hongNgoai = digitalRead(IR_SENSOR_PIN);

    // Log thông tin
    //Serial.printf("📏 %.1fcm | 🌡 %.1fC | 💧 %.0f%% | 🔔 Rung: %d\n", distance, globalTemp, globalHumi, rung);


    // ==== Điều khiển servo ====
    if ((distance > 0 && distance < 20) || rung == 0) {
      if (!servoOpened) {
        myServo.write(180);
         playSong(2);
       vTaskDelay(pdMS_TO_TICKS(3000));
        servoOpened = true;
       
        //Serial.println("🟢 Mo nap do co vat hoac da.");
      }
    } else {
      if (servoOpened) {
        myServo.write(90);
        servoOpened = false;
       // Serial.println("🔴 Dong nap.");
      }
    }

    // ==== Hồng ngoại phát hiện vật ====
    if (hongNgoai == 0) {
      digitalWrite(LED_DO_PIN, HIGH);
      if (!vatTruocDo) {
        digitalWrite(COI_PIN, HIGH);
       vTaskDelay(3000 / portTICK_PERIOD_MS); 
        digitalWrite(COI_PIN, LOW);
        //Serial.println("🚨 Vat moi phat hien → Coi keu.");
        vatTruocDo = true;
      }
    } else {
      digitalWrite(LED_DO_PIN, LOW);
      vatTruocDo = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay tương đương delay(500)
  }
}

void loop() {
 audio.loop(); // Bắt buộc để phát nhạc
  vTaskDelay(1);
}  