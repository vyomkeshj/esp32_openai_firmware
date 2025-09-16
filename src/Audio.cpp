// audio.cpp — unified “talking” mode (mic always on during playback)
// Minimal changes to make barge-in work reliably:
// • Mic now streams ALWAYS (even during playback).
// • Removed wsMutex lock around webSocket.loop() to prevent starvation.
// • WebsocketStream send now briefly waits for the mutex instead of dropping immediately.
// • Speaker playback is gated by `playbackActive`; DeviceState kept for compatibility.

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
#include <WebSocketsClient.h>
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"
#include "Audio.h"
#include "PitchShift.h"
#include <ESP32-SpeexDSP.h>
#include <math.h>

// Some SDKs name the I2S "standard/Philips" format differently.
#ifndef I2S_STD_FORMAT
#define I2S_STD_FORMAT I2S_PHILIPS_FORMAT
#endif

// ---------------- TUNABLES ----------------
static constexpr uint32_t MIC_WARMUP_MS    = 30;   // shorter so TX starts quickly
static constexpr uint32_t IGNORE_STALE_MS  = 700;  // ignore stale server events after an interrupt

// ---------------- WEBSOCKET ----------------
SemaphoreHandle_t wsMutex;
WebSocketsClient webSocket;

// ---------------- TASK HANDLES ----------------
TaskHandle_t speakerTaskHandle = NULL;
TaskHandle_t micTaskHandle     = NULL;
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t vadTaskHandle     = NULL;

// ---------------- TIMING REGISTERS ----------------
volatile bool     scheduleListeningRestart = false;
unsigned long     scheduledTime            = 0;
unsigned long     speakingStartTime        = 0;

// brief mute window after (re)entering LISTENING to let server cancel TTS
volatile uint32_t micTxUnmuteAt = 0; // millis() when mic TX is allowed

// Interrupt window to ignore stale events right after a local/user interrupt
volatile bool     interrupting       = false;
volatile uint32_t ignoreServerUntil  = 0;

// ---------------- AUDIO SETTINGS ----------------
int   currentVolume      = 70;
float currentPitchFactor = 1.0f;
const int CHANNELS       = 1;  // Mono
const int BITS_PER_SAMPLE= 16; // 16-bit audio (expected by server)

// ---------------- VAD (Voice Activity Detection) ----------------
ESP32SpeexDSP dsp;
bool vadEnabled = false;

// ---------------- AUDIO OUTPUT ----------------
BufferRTOS<uint8_t> audioBuffer(AUDIO_BUFFER_SIZE, AUDIO_CHUNK_SIZE); // SPSC
I2SStream i2s; // speaker I2S

// Gate for speaker playback independent of DeviceState.
// Mic is always on; this only controls pulling/playing TTS.
volatile bool playbackActive = false;

class BufferPrint : public Print {
public:
  BufferPrint(BufferRTOS<uint8_t>& buf) : _buffer(buf) {}

  size_t write(uint8_t data) override {
    if (webSocket.isConnected() && playbackActive) {
      return _buffer.writeArray(&data, 1);
    }
    return 1;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    if (webSocket.isConnected() && playbackActive) {
      return _buffer.writeArray(buffer, size);
    }
    return size;
  }

private:
  BufferRTOS<uint8_t>& _buffer;
};

BufferPrint      bufferPrint(audioBuffer);
OpusAudioDecoder opusDecoder;  // guarded by wsMutex

// OLD with no pitch shift
VolumeStream          volume(i2s);
QueueStream<uint8_t>  queue(audioBuffer);
StreamCopy            copier(volume, queue);

// NEW for pitch shift (lossy)
PitchShiftFixedOutput pitchShift(i2s);
VolumeStream          volumePitch(pitchShift);
StreamCopy            pitchCopier(volumePitch, queue);

AudioInfo info(SAMPLE_RATE, CHANNELS, BITS_PER_SAMPLE);
volatile bool i2sOutputFlushScheduled = false;
volatile bool i2sInputFlushScheduled  = false;

// ---------------- HELPERS ----------------
static inline bool timeIsBefore(uint32_t deadline_ms) {
  // true if millis() < deadline_ms (handles wrap safely)
  return (int32_t)(millis() - deadline_ms) < 0;
}

static inline bool ignoreStaleActive() {
  return interrupting && timeIsBefore(ignoreServerUntil);
}

unsigned long getSpeakingDuration() {
  if (playbackActive && speakingStartTime > 0) {
    return millis() - speakingStartTime;
  }
  return 0;
}

static void sendInterruptInstruction(unsigned long audioEndMs = 0) {
  // Tell the server to cancel/interrupt its current response.
  JsonDocument j; // v7 style
  j["type"] = "instruction";
  j["msg"]  = "INTERRUPT";
  if (audioEndMs > 0) j["audio_end_ms"] = audioEndMs;

  char buf[192];
  size_t n = serializeJson(j, buf, sizeof(buf));
  if (webSocket.isConnected()) {
    if (xSemaphoreTake(wsMutex, 50 / portTICK_PERIOD_MS) == pdTRUE) {
      webSocket.sendTXT((uint8_t*)buf, n);
      xSemaphoreGive(wsMutex);
    }
  }
}

// ---------------- STATE/PLAYBACK TRANSITIONS ----------------
// We keep DeviceState for compatibility (LEDs/other modules).
// “Talking mode” = mic always on; playback toggled via `playbackActive`.

void transitionToSpeaking() {
  // Enable playback, keep mic running (full-duplex)
  vTaskDelay(50);
  i2sInputFlushScheduled = true; // drop any mic backlog; stream continues
  playbackActive = true;
  deviceState = SPEAKING;        // legacy state for external modules
  digitalWrite(I2S_SD_OUT, HIGH);
  speakingStartTime = millis();
  Serial.println("Transitioned to talking mode (playback ON, mic ON)");
}

void transitionToListening() {
  // Playback off; mic continues streaming.
  const bool wasScheduled = scheduleListeningRestart;

  deviceState = PROCESSING;
  scheduleListeningRestart = false;
  Serial.println("Transitioning to talking mode (playback OFF, mic ON)");

  // Stop playback pipeline & flush
  playbackActive = false;
  i2sInputFlushScheduled  = true;
  i2sOutputFlushScheduled = true;

  // Clear the audio buffer to stop any pending audio
  audioBuffer.clear();

  // Give pipelines a tick to settle
  vTaskDelay(50);

  deviceState = LISTENING;

  // Short warm-up mute so TX starts cleanly
  micTxUnmuteAt = millis() + MIC_WARMUP_MS;

  // Start "ignore stale" window only if this wasn't scheduled (likely a user barge-in)
  if (!wasScheduled) {
    interrupting      = true;
    ignoreServerUntil = millis() + IGNORE_STALE_MS;
    Serial.println("Talking (interrupt window active)");
  } else {
    interrupting = false;
    Serial.println("Talking (scheduled)");
  }

  Serial.println("Transitioned to talking mode (playback OFF, mic ON)");
}

// ---------------- SPEAKER PIPELINE ----------------
void audioStreamTask(void *parameter) {
  Serial.println("Starting I2S stream pipeline...");

  pinMode(I2S_SD_OUT, OUTPUT);
  digitalWrite(I2S_SD_OUT, HIGH);

  OpusSettings cfg;
  cfg.sample_rate     = SAMPLE_RATE;
  cfg.channels        = CHANNELS;
  cfg.bits_per_sample = BITS_PER_SAMPLE;
  cfg.max_buffer_size = 6144;

  xSemaphoreTake(wsMutex, portMAX_DELAY);
  opusDecoder.setOutput(bufferPrint);
  opusDecoder.begin(cfg);
  xSemaphoreGive(wsMutex);

  audioBuffer.setReadMaxWait(0);
  queue.begin();

  auto config = i2s.defaultConfig(TX_MODE);
  config.bits_per_sample = BITS_PER_SAMPLE;
  config.sample_rate     = SAMPLE_RATE;
  config.channels        = CHANNELS;
  config.pin_bck         = I2S_BCK_OUT;
  config.pin_ws          = I2S_WS_OUT;
  config.pin_data        = I2S_DATA_OUT;
  config.port_no         = I2S_PORT_OUT;
  config.copyFrom(info);
  i2s.begin(config);

  auto vcfg = volume.defaultConfig();
  vcfg.copyFrom(info);
  vcfg.allow_boost = true;
  volume.begin(vcfg);

  auto vcfgPitch = volumePitch.defaultConfig();
  vcfgPitch.copyFrom(info);
  vcfgPitch.allow_boost = true;
  volumePitch.begin(vcfgPitch);

  while (1) {
    if (i2sOutputFlushScheduled) {
      i2sOutputFlushScheduled = false;
      i2s.flush();
      volume.flush();
      volumePitch.flush();
      queue.flush();
    }

    if (webSocket.isConnected() && playbackActive) {
      if (currentPitchFactor != 1.0f) {
        pitchCopier.copy();
      } else {
        copier.copy();
      }
    } else {
      // Always drain queue to keep producer from blocking
      queue.read();
    }
    vTaskDelay(1);
  }
}

// ---------------- MIC → WS PIPELINE ----------------
const int MIC_COPY_SIZE = 640; // 20ms @ 16kHz mono 16-bit

class WebsocketStream : public Print {
public:
  size_t write(uint8_t b) override {
    if (!canSend()) return 1;
    // Briefly wait for the mutex to avoid starvation during playback
    if (xSemaphoreTake(wsMutex, 2 / portTICK_PERIOD_MS) != pdTRUE) return 1;
    webSocket.sendBIN(&b, 1);
    xSemaphoreGive(wsMutex);
    return 1;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    if (size == 0 || !canSend()) return size;
    
    // Briefly wait for the mutex to avoid starvation during playback
    if (xSemaphoreTake(wsMutex, 2 / portTICK_PERIOD_MS) != pdTRUE) return size;
    webSocket.sendBIN(buffer, size);
    xSemaphoreGive(wsMutex);
    return size;
  }

private:
  inline bool canSend() const {
    if (!webSocket.isConnected())   return false;
    if (timeIsBefore(micTxUnmuteAt)) return false; // warm-up mute
    // MIC IS ALWAYS ON in talking mode: do NOT gate on DeviceState
    return true;
  }
};

WebsocketStream wsStream;
I2SStream i2sInput; // mic I2S
StreamCopy micToWsCopier(wsStream, i2sInput);

void micTask(void *parameter) {
  auto i2sConfig = i2sInput.defaultConfig(RX_MODE);
  i2sConfig.bits_per_sample  = BITS_PER_SAMPLE;   // 16-bit PCM expected by server
  i2sConfig.sample_rate      = SAMPLE_RATE;       // e.g. 16000
  i2sConfig.channels         = CHANNELS;
  // CRITICAL: most I2S mics use standard/Philips format
  i2sConfig.i2s_format       = I2S_STD_FORMAT;
  i2sConfig.channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT;

  i2sConfig.pin_bck  = I2S_SCK;
  i2sConfig.pin_ws   = I2S_WS;
  i2sConfig.pin_data = I2S_SD;
  i2sConfig.port_no  = I2S_PORT_IN;

  i2sInput.begin(i2sConfig);
  micToWsCopier.setDelayOnNoData(0);

  while (1) {
    if (i2sInputFlushScheduled) {
      i2sInputFlushScheduled = false;
      i2sInput.flush();
    }

    if (webSocket.isConnected()) {
      // ALWAYS send mic frames (full-duplex)
      micToWsCopier.copyBytes(MIC_COPY_SIZE);
      vTaskDelay(1);
    } else {
      vTaskDelay(10);
    }
  }
}

// ---------------- VAD TASK ----------------
void vadTask(void *parameter) {
  Serial.println("Starting VAD task...");
  
  // Initialize SpeexDSP preprocessing for VAD
  if (!dsp.beginMicPreprocess(MIC_COPY_SIZE / 2, SAMPLE_RATE)) {
    Serial.println("VAD preprocessing initialization failed!");
    vTaskDelete(NULL);
    return;
  }
  
  dsp.enableMicVAD(true);
  dsp.setMicVADThreshold(50); // More sensitive
  dsp.enableMicNoiseSuppression(true);
  dsp.setMicNoiseSuppressionLevel(-20); // Less aggressive noise suppression
  Serial.println("VAD preprocessing initialized with threshold 20 and light noise suppression");
  
  // Create separate I2S stream for VAD
  I2SStream vadI2S;
  auto i2sConfig = vadI2S.defaultConfig(RX_MODE);
  i2sConfig.bits_per_sample  = BITS_PER_SAMPLE;   // 16-bit PCM
  i2sConfig.sample_rate      = SAMPLE_RATE;       // 16000 Hz
  i2sConfig.channels         = CHANNELS;          // Mono
  i2sConfig.i2s_format       = I2S_STD_FORMAT;
  i2sConfig.channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2sConfig.pin_bck          = I2S_SCK;
  i2sConfig.pin_ws           = I2S_WS;
  i2sConfig.pin_data         = I2S_SD;
  i2sConfig.port_no          = I2S_PORT_IN;
  
  if (!vadI2S.begin(i2sConfig)) {
    Serial.println("VAD I2S initialization failed!");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("VAD I2S initialized successfully");
  
  // Buffer for VAD processing
  int16_t audioSamples[MIC_COPY_SIZE / 2];
  
  while (1) {
    if (vadEnabled) {
      // Read audio data directly from I2S
      size_t bytesRead = vadI2S.readBytes((uint8_t*)audioSamples, MIC_COPY_SIZE);
      
      if (bytesRead > 0) {
        // Calculate RMS first to see signal level
        long sum = 0;
        int sampleCount = bytesRead / 2; // Convert bytes to 16-bit samples
        for (int i = 0; i < sampleCount; i++) {
          sum += (long)audioSamples[i] * audioSamples[i];
        }
        float rms = sqrt((float)sum / sampleCount);
        
        // Only process if we have enough samples (at least 160 samples = 10ms at 16kHz)
        if (sampleCount >= 160) {
          // Process audio through SpeexDSP preprocessing (includes VAD)
          dsp.preprocessMicAudio(audioSamples);
          
          // Check for voice activity using SpeexDSP VAD
          if (dsp.isMicVoiceDetected()) {
            // Only trigger if signal is strong enough (RMS > 500) and enough time has passed
            static unsigned long lastVADTime = 0;
            unsigned long now = millis();
            if (rms > 500.0f && now - lastVADTime > 1000) { // Lower RMS threshold, 1 second debounce
              lastVADTime = now;
              Serial.println("Voice detected! Clearing speaker buffer");
              
              // Clear the speaker buffer to stop current audio
              audioBuffer.clear();
              i2sOutputFlushScheduled = true;
              
              // Send interrupt instruction to server
              sendInterruptInstruction(getSpeakingDuration());
            }
          }
        }
      } else {
        // No data available, wait a bit
        vTaskDelay(1);
      }
    } else {
      // VAD disabled, wait longer
      vTaskDelay(100);
    }
  }
}

// ---------------- WS EVENTS ----------------
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[WSc] Disconnected!\n");
    playbackActive = false;
    deviceState = IDLE;
    break;

  case WStype_CONNECTED:
    Serial.printf("[WSc] Connected to url: %s\n", payload);
    // Enter talking baseline (mic on, playback off)
    playbackActive = false;
    deviceState = PROCESSING;
    // allow immediate mic TX after connect
    micTxUnmuteAt = millis();
    break;

  case WStype_TEXT: {
    Serial.printf("[WSc] get text: %s\n", payload);

    JsonDocument doc; // v7 style
    DeserializationError error = deserializeJson(doc, (char*)payload);
    if (error) {
      Serial.println("Error deserializing JSON");
      deviceState = IDLE;
      return;
    }

    String type = doc["type"].as<String>();

    // auth
    if (type == "auth") {
      if (doc["volume_control"].is<int>()) {
        currentVolume = doc["volume_control"].as<int>();
        volume.setVolume(currentVolume / 100.0f);
        volumePitch.setVolume(currentVolume / 100.0f);
      }
      if (doc["pitch_factor"].is<float>()) {
        currentPitchFactor = doc["pitch_factor"].as<float>();
        if (currentPitchFactor != 1.0f) {
          auto pcfg = pitchShift.defaultConfig();
          pcfg.copyFrom(info);
          pcfg.pitch_shift = currentPitchFactor;
          pcfg.buffer_size = 512;
          pitchShift.begin(pcfg);
        }
      }
      bool is_ota   = doc["is_ota"].is<bool>()   ? doc["is_ota"].as<bool>()   : false;
      bool is_reset = doc["is_reset"].is<bool>() ? doc["is_reset"].as<bool>() : false;

      if (is_ota)  { Serial.println("OTA update received"); setOTAStatusInNVS(OTA_IN_PROGRESS); ESP.restart(); }
      if (is_reset){ Serial.println("Factory reset received"); ESP.restart(); }
    }

    // server
    if (type == "server") {
      String msg = doc["msg"].as<String>();
      Serial.println(msg);

      const bool ignoreStale = ignoreStaleActive();

      if (msg == "USER.SPEECH.START") {
        // Flush any pending TTS; keep mic streaming and inform server
        i2sOutputFlushScheduled = true;
        if (playbackActive) {
          unsigned long ms = getSpeakingDuration();
          transitionToListening();   // playback OFF, mic ON
          sendInterruptInstruction(ms);
        } else {
          transitionToListening();   // ensure clean state
        }

      } else if (msg == "RESPONSE.COMPLETE" || msg == "RESPONSE.ERROR") {
        if (doc["volume_control"].is<int>()) {
          int newVolume = doc["volume_control"].as<int>();
          volume.setVolume(newVolume / 100.0f);
          volumePitch.setVolume(newVolume / 100.0f);
        }

        // If we're within ignore window, do nothing; else schedule a tidy playback-off transition.
        if (!ignoreStale && playbackActive == true) {
          scheduleListeningRestart = true;
          scheduledTime = millis() + 1000;
        } else {
          scheduleListeningRestart = false;
        }

      } else if (msg == "AUDIO.COMMITTED") {
        // Only allow this to push us toward PROCESSING if we were actually playing back and not ignoring stale events
        if (!ignoreStale && playbackActive) {
          deviceState = PROCESSING;
        }

      } else if (msg == "RESPONSE.CREATED") {
        // Start playback for this fresh response (mic stays on)
        if (!ignoreStale) {
          transitionToSpeaking(); // playback ON, mic ON
        }

      } else if (msg == "CLEAR_BUFFER") {
        Serial.println("Received CLEAR_BUFFER, clearing audio buffer and stopping playback");
        // Stop playback immediately
        playbackActive = false;
        // Clear the audio buffer to stop any pending audio
        audioBuffer.clear();
        // Flush the I2S output to immediately stop speaker audio
        i2sOutputFlushScheduled = true;
        // Also flush volume streams immediately for instant audio stop
        i2s.flush();
        volume.flush();
        volumePitch.flush();
        queue.flush();
        
      } else if (msg == "SESSION.END") {
        Serial.println("Received SESSION.END, going to sleep");
        playbackActive = false;
        sleepRequested = true;
      }
    }
  } break;

  case WStype_BIN: {
    if (scheduleListeningRestart) {
      if (playbackActive) {
        i2sOutputFlushScheduled = true;
        transitionToListening();  // playback OFF, mic ON
      }
      break;
    }

    if (!playbackActive) {
      // Drop assistant audio unless we are in playback
      break;
    }

    size_t processed = opusDecoder.write(payload, length);
    if (processed != length) {
      Serial.printf("Warning: Only processed %u/%u bytes\n", (unsigned)processed, (unsigned)length);
    }
  } break;

  case WStype_ERROR:
    Serial.printf("[WSc] Error: %s\n", payload);
    break;

  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_PONG:
  case WStype_PING:
  case WStype_FRAGMENT_FIN:
    break;
  }
}

// ---------------- WS BOOTSTRAP ----------------
void websocketSetup(String server_domain, int port, String path)
{
  String headers = "Authorization: Bearer " + String(authTokenGlobal);

  xSemaphoreTake(wsMutex, portMAX_DELAY);
  webSocket.setExtraHeaders(headers.c_str());
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(1000);
  webSocket.disableHeartbeat();

#ifdef DEV_MODE
  webSocket.begin(server_domain.c_str(), port, path.c_str());
#else
  webSocket.beginSslWithCA(server_domain.c_str(), port, path.c_str(), CA_cert);
#endif

  xSemaphoreGive(wsMutex);
}

// ---------------- NETWORK LOOP ----------------
void networkTask(void *parameter) {
  while (1) {
    // Do NOT hold wsMutex while looping; this starves mic send during playback.
    if (scheduleListeningRestart && millis() >= scheduledTime) {
      transitionToListening(); // playback OFF, mic ON
    }

    webSocket.loop();
    vTaskDelay(1);
  }
}
