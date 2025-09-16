#include "OTA.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include "LEDHandler.h"
#include "Config.h"
#include "SPIFFS.h"
#include "WifiManager.h"
#include <driver/touch_sensor.h>
#include "Button.h"
#include "FactoryReset.h"
#include <ArduinoJson.h>

#define TOUCH_THRESHOLD 28000
#define REQUIRED_RELEASE_CHECKS 100
#define TOUCH_DEBOUNCE_DELAY 500 // ms

AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;

void enterSleep()
{
    Serial.println("Going to sleep...");

    deviceState = SLEEP;
    scheduleListeningRestart = false;
    i2sOutputFlushScheduled = true;
    i2sInputFlushScheduled = true;
    vTaskDelay(10);

    xSemaphoreTake(wsMutex, portMAX_DELAY);
    i2s_stop(I2S_PORT_IN);
    i2s_stop(I2S_PORT_OUT);

    if (webSocket.isConnected()) {
        webSocket.disconnect();
    }
    xSemaphoreGive(wsMutex);
    delay(100);

    i2s_driver_uninstall(I2S_PORT_IN);
    i2s_driver_uninstall(I2S_PORT_OUT);

    Serial.flush();

#ifdef TOUCH_MODE
    touch_pad_intr_disable(TOUCH_PAD_INTR_MASK_ALL);
    while (touchRead(TOUCH_PAD_NUM2) > TOUCH_THRESHOLD) { delay(50); }
    delay(500);
    touchSleepWakeUpEnable(TOUCH_PAD_NUM2, TOUCH_THRESHOLD);
#endif

    esp_deep_sleep_start();
    delay(1000);
}

void processSleepRequest() {
    if (sleepRequested) {
        sleepRequested = false;
        enterSleep();
    }
}

void printOutESP32Error(esp_err_t err)
{
    switch (err) {
    case ESP_OK: Serial.println("ESP_OK no errors"); break;
    case ESP_ERR_INVALID_ARG: Serial.println("ESP_ERR_INVALID_ARG if the selected GPIO is not an RTC GPIO, or the mode is invalid"); break;
    case ESP_ERR_INVALID_STATE: Serial.println("ESP_ERR_INVALID_STATE if wakeup triggers conflict or wireless not stopped"); break;
    default: Serial.printf("Unknown error code: %d\n", err); break;
    }
}

static void onButtonLongPressUpEventCb(void *button_handle, void *usr_data)
{
    Serial.println("Button long press end");
    delay(10);
    sleepRequested = true;
}

static void onButtonDoubleClickCb(void *button_handle, void *usr_data)
{
    Serial.println("Button double click");
    delay(10);
    sleepRequested = true;
}

// Single-click: interrupt TTS, flush output, go LISTENING, and notify server
static void onButtonSingleClickCb(void *button_handle, void *usr_data)
{
    static uint32_t lastClickMs = 0;                 // tiny debounce
    uint32_t now = millis();
    if (now - lastClickMs < 300) return;
    lastClickMs = now;

    Serial.println("Button single click");
    delay(10);

    if (deviceState == LISTENING) return;

    i2sOutputFlushScheduled = true;
    i2sInputFlushScheduled = true;
    scheduleListeningRestart = false;

    if (webSocket.isConnected()) {
        JsonDocument j; // v7 style
        j["type"] = "instruction";
        j["msg"]  = "INTERRUPT";

        extern unsigned long getSpeakingDuration();
        if (deviceState == SPEAKING) {
            unsigned long ms = getSpeakingDuration();
            if (ms > 0) j["audio_end_ms"] = ms;
        }

        char buf[192];
        size_t n = serializeJson(j, buf, sizeof(buf));
        xSemaphoreTake(wsMutex, portMAX_DELAY);
        webSocket.sendTXT((uint8_t*)buf, n);
        xSemaphoreGive(wsMutex);
    }

    extern void transitionToListening();
    transitionToListening();
}

void getAuthTokenFromNVS()
{
    preferences.begin("auth", false);
    authTokenGlobal = preferences.getString("auth_token", "");
    preferences.end();
}

void setupWiFi()
{
    WifiManager.startBackgroundTask("Bubbi-DEVICE");
    WifiManager.fallbackToSoftAp(true);
    WifiManager.attachWebServer(&webServer);
    WifiManager.attachUI();

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/wifi");
    });
    webServer.onNotFound([&](AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "Not found");
    });
    webServer.begin();
}

void touchTask(void* parameter) {
  touch_pad_init();
  touch_pad_config(TOUCH_PAD_NUM2);

  bool touched = false;
  unsigned long pressStartTime = 0;
  unsigned long lastTouchTime = 0;
  const unsigned long LONG_PRESS_DURATION = 500;

  while (1) {
    uint32_t touchValue = touchRead(TOUCH_PAD_NUM2);
    bool isTouched = (touchValue > TOUCH_THRESHOLD);
    unsigned long currentTime = millis();

    if (isTouched && !touched && (currentTime - lastTouchTime > TOUCH_DEBOUNCE_DELAY)) {
      touched = true;
      pressStartTime = currentTime;
      lastTouchTime = currentTime;
    }

    if (touched && isTouched) {
      if (currentTime - pressStartTime >= LONG_PRESS_DURATION) {
        sleepRequested = true;
      }
    }

    if (!isTouched && touched) {
      touched = false;
      pressStartTime = 0;
    }

    vTaskDelay(20);
  }
  vTaskDelete(NULL);
}

void setupDeviceMetadata() {
    deviceState = IDLE;

    getAuthTokenFromNVS();
    getOTAStatusFromNVS();

    if (otaState == OTA_IN_PROGRESS || otaState == OTA_COMPLETE) {
        deviceState = OTA;
    }
    if (factory_reset_status) {
        deviceState = FACTORY_RESET;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    setupDeviceMetadata();
    wsMutex = xSemaphoreCreateMutex();

#ifdef TOUCH_MODE
    xTaskCreate(touchTask, "Touch Task", 4096, NULL, configMAX_PRIORITIES-2, NULL);
#else
    getErr = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, LOW);
    printOutESP32Error(getErr);
    Button *btn = new Button(BUTTON_PIN, false);
    // btn->attachLongPressUpEventCb(&onButtonLongPressUpEventCb, NULL);
    // btn->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
    // btn->detachSingleClickEvent();
    btn->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
#endif

    xTaskCreatePinnedToCore(ledTask,         "LED Task",        4096, NULL, 5,  NULL, 1);
    xTaskCreatePinnedToCore(audioStreamTask, "Speaker Task",    4096, NULL, 3,  NULL, 1);
    xTaskCreatePinnedToCore(micTask,         "Microphone Task", 4096, NULL, 4,  NULL, 1);
    xTaskCreatePinnedToCore(networkTask,     "Websocket Task",  8192, NULL, configMAX_PRIORITIES-1, &networkTaskHandle, 0);

    setupWiFi();
}

void loop(){
    processSleepRequest();
    if (otaState == OTA_IN_PROGRESS) {
        loopOTA();
    }
}
