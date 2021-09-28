#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "driver/rtc_io.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <WebSerial.h>
#include "esp_camera.h"

/*
   Error Codes (number of consecutive flashes of red light):
   2: Error initializing preferences
   3: Error initializing camera
   4: Error initializing SD card
   5: Error initializing MPU6050
   6: Error initializing BMP280
   7: Error reading frame
   8: Error opening video file
   9: Error opening data file
*/

#define SCL_PIN 3
#define SDA_PIN 1
#define VERSION 7
#define NAME "RocketCam"

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

TaskHandle_t readCameraTask;
TaskHandle_t writeVideoTask;
TaskHandle_t readSensorTask;
Preferences preferences;
AsyncWebServer server(80);
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

char videoFileName[64];
char dataFileName[64];

bool debugMode = false;
int runCount = 0;
camera_fb_t* fbQueue[5] = {NULL, NULL, NULL, NULL, NULL};
volatile bool waitingToStart = false;
volatile bool recording = true;
volatile bool queueFull[5] = {false, false, false, false, false};
volatile uint16_t frameCount = 0;


void showError(byte code) {
  digitalWrite(33, HIGH);
  delay(1000);

  while (true) {
    if (debugMode) WebSerial.println("Error code: " + String(code));
    for (byte j = 0; j < code; j++) {
      digitalWrite(33, LOW);
      delay(200);
      digitalWrite(33, HIGH);
      delay(300);
    }
    delay(1000);
  }
}

void initPreferences() {
  if (!preferences.begin(NAME, false)) {
    showError(2);
  }

  if (preferences.getBool("debug", false)) { //Setup did not finish last time
    preferences.putBool("debug", false); //Boot normally next time
    debugMode = true;
  } else { //Normal boot
    preferences.putBool("debug", true); //This should be reset after setup. If error occurs in setup, boot in debug next time
  }

  runCount = preferences.getInt("run", 0);
  if (preferences.getInt("version", 0) != VERSION) {
    preferences.putInt("version", VERSION);
    preferences.putInt("run", 1);
    runCount = 1;
  }

  preferences.end();

  sprintf(videoFileName, "/sdcard/videos/video%03i.%03i.jpgs",
          VERSION,
          runCount);
  sprintf(dataFileName, "/sdcard/data/log%03i.%03i.csv",
          VERSION,
          runCount);
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (!psramFound()) {
    showError(10);
  }
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 5;
  config.fb_count = 7;

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  
  //https://github.com/espressif/esp32-camera/issues/185
  //Set lower quality after initializing buffers
  //Should fix cut-off images in high density pictures
  sensor_t * s = esp_camera_sensor_get();
  s->set_quality(s, 12);
  delay(500);
  
  if (err != ESP_OK) {
    if (debugMode) WebSerial.println("Camera init failed with error " + String(err));
    showError(3);
  }
}

void initSD() {
  pinMode(13, INPUT_PULLUP);

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;                       // using 1 bit mode
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  slot_config.width = 1;                                   // using 1 bit mode
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 8,
  };
  sdmmc_card_t *card;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
  
  if (ret != ESP_OK) {
    if (debugMode) WebSerial.println("Failed to mount SD card VFAT filesystem. Error: " + String(esp_err_to_name(ret)));
    showError(4);
  }

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}

void initMPU() {
  if (!mpu.begin()) {
    showError(5);
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void initBMP() {
  if (!bmp.begin(0x76)) {
    showError(6);
    return;
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);
}

void readCamera(void* pvParameters) {
  byte queuePos = 0;

  while (true) {
    if (recording && !queueFull[queuePos]) {
      fbQueue[queuePos] = esp_camera_fb_get();
      if (!fbQueue[queuePos]) {
        showError(7);
      }

      queueFull[queuePos] = true;
      queuePos = (queuePos + 1) % 5;
      frameCount++;
    }
    delay(1);
  }
}

void writeVideo(void* pvParameters) {
  byte queuePos = 0;
  unsigned long i = 1;

  FILE* file = fopen(videoFileName, "w");
  if (file == NULL) {
    showError(8);
  }

  while (true) {
    if (queueFull[queuePos]) {
      camera_fb_t* fb = fbQueue[queuePos];
      uint8_t len[4];
      len[0] = (fb->len >> 0) & 0xff;
      len[1] = (fb->len >> 8) & 0xff;
      len[2] = (fb->len >> 16) & 0xff;
      len[3] = (fb->len >> 24) & 0xff;

      fwrite(len, 1, 4, file);
      fwrite(fb->buf, 1, fb->len, file);

      if (i % 20 == 0) {
        fflush(file);
        fsync(fileno(file));
        digitalWrite(4, HIGH);
        delayMicroseconds(500);
        digitalWrite(4, LOW);
      }

      esp_camera_fb_return(fb);

      queueFull[queuePos] = false;
      queuePos = (queuePos + 1) % 5;
      i++;
    }
    delay(1);
  }
}

void readSensor(void* pvParameters) {
  unsigned long lastRead = millis();
  char buff[128];
  unsigned long i = 1;

  FILE* file = fopen(dataFileName, "w");
  if (file == NULL) {
    showError(9);
  }

  size_t chars = sprintf(buff, "ms,Frame,Ax,Ay,Az,Gx,Gy,Gz,Press,Temp\n");
  fwrite(buff, 1, chars, file);

  while (true) {
    if (recording && millis() - lastRead >= 50) {
      lastRead += 50;

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float pressure = bmp.readPressure();
      float temperature = bmp.readTemperature();

      size_t chars = sprintf(buff, "%i,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
                             millis(),
                             frameCount,
                             a.acceleration.x, a.acceleration.y, a.acceleration.z,
                             g.gyro.x, g.gyro.y, g.gyro.z,
                             pressure,
                             temperature);

      fwrite(buff, 1, chars, file);

      if (i % 10 == 0) {
        fflush(file);
        fsync(fileno(file));
      }

      i++;
    }
    delay(1);
  }
}

void startSDServer() {
  if (!SD_MMC.begin("/sdcard", true)) {
    WebSerial.println("Error initializing SD card.");
    return;
  }

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  server.serveStatic("/sd/", SD_MMC, "/");
  WebSerial.println("SD server started. Go to <ip>/sd/data.txt");
}

void webSerialMsg(uint8_t* data, size_t len) {
  String dataStr = "";
  for (int i = 0; i < len; i++) dataStr += (char)data[i];

  if (dataStr == "start") {
    WebSerial.println("Video file " + String(videoFileName));
    WebSerial.println("Data file " + String(dataFileName));
    WebSerial.println("Starting...\n");
    waitingToStart = false;
    recording = true;

  } else if (dataStr == "stop") {
    WebSerial.println("Recording stopped.");
    recording = false;

  } else if (dataStr == "server") {
    startSDServer();

  } else if (dataStr == "help") {
    WebSerial.println("\nVersion " + String(VERSION));
    WebSerial.println("Video file: " + String(videoFileName));
    WebSerial.println("Data file:  " + String(dataFileName));
    WebSerial.println("Available commands:");
    WebSerial.println("  start - Start recording normally");
    WebSerial.println("  stop - Stop recording");
    WebSerial.println("  server - Serve files from SD card");
    WebSerial.println("  help - Print this\n");

  } else {
    WebSerial.println("Unknown command '" + dataStr + "'");
  }
}

void setup() {
  pinMode(33, OUTPUT);

  initPreferences();

  digitalWrite(33, LOW); //turn on led

  if (debugMode) {
    delay(50);
    digitalWrite(33, HIGH);

    WiFi.mode(WIFI_AP); //Access point. Default IP: 192.168.4.1
    WiFi.softAP("ESP32Camera");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Hi! I am the ESP32.");
    });

    AsyncElegantOTA.begin(&server);

    WebSerial.begin(&server);
    WebSerial.msgCallback(webSerialMsg);

    server.begin();

    //Wait to finish setup until webserial is connected
    //This way, any error messages can be seen.
    waitingToStart = true;
    while (waitingToStart) delay(500);
  }


  //================ Main Setup Code ======================
  delay(500);

  initCamera();
  if (debugMode) WebSerial.println("Camera initialized");

  initSD();
  if (debugMode) WebSerial.println("SD card initialized");

  Wire.begin(SDA_PIN, SCL_PIN, 20000);
  initMPU();
  if (debugMode) WebSerial.println("MPU6050 initialized");

  initBMP();
  if (debugMode) WebSerial.println("BMP280 initialized");

//  if (debugMode) {
//    WebSerial.println(ESP.getHeapSize());
//    WebSerial.println(ESP.getFreeHeap());
//    WebSerial.println(ESP.getPsramSize());
//    WebSerial.println(ESP.getFreePsram());
//  }

  xTaskCreatePinnedToCore(
    readCamera, /* Function to implement the task */
    "readCamera", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    9,  /* Priority of the task */
    &readCameraTask,  /* Task handle. */
    0); /* Core where the task should run */

  xTaskCreatePinnedToCore(
    writeVideo, /* Function to implement the task */
    "writeVideo", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    9,  /* Priority of the task */
    &writeVideoTask,  /* Task handle. */
    1); /* Core where the task should run */

  xTaskCreatePinnedToCore(
    readSensor, /* Function to implement the task */
    "readSensor", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    9,  /* Priority of the task */
    &readSensorTask,  /* Task handle. */
    0); /* Core where the task should run */

  if (debugMode) WebSerial.println("Tasks started");
  //================ End Main Setup Code ==================


  digitalWrite(33, HIGH);
  delay(500);
  
  preferences.begin(NAME, false);
  preferences.putInt("run", runCount + 1);
  preferences.putBool("debug", false); //Setup finished, boot normally next time
  preferences.end();
}

void loop() { }
