#include "OTA.h"
#include "Print.h"
#include "Config.h"
#include "AudioTools.h"
// #include "AudioTools/Concurrency/RTOS.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"
#include <WebSocketsClient.h>
#include "Audio.h"
#include "PitchShift.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <esp_freertos_hooks.h>
#include <esp_timer.h>

// WEBSOCKET
SemaphoreHandle_t wsMutex;
WebSocketsClient webSocket;

// TASK HANDLES
TaskHandle_t speakerTaskHandle = NULL;
TaskHandle_t micTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;

// TIMING REGISTERS
volatile bool scheduleListeningRestart = false;
unsigned long scheduledTime = 0;
unsigned long speakingStartTime = 0;

// AUDIO SETTINGS
int currentVolume = 70;
float currentPitchFactor = 1.0f;
const int CHANNELS = 1;         // Mono
const int BITS_PER_SAMPLE = 16; // 16-bit audio

// AUDIO OUTPUT
class BufferPrint : public Print {
public:
  BufferPrint(BufferRTOS<uint8_t>& buf) : _buffer(buf) {}

  // networkTask -> webSocket.loop() -> webSocketEvent(WStype_BIN, ...) -> opusDecoder.write() -> bufferPrint.write()
  virtual size_t write(uint8_t data) override {
    if (webSocket.isConnected()) {
        return _buffer.writeArray(&data, 1);
    }
    return 1; //let opusDecoder write, otherwise thread will stuck
  }

  // networkTask -> webSocket.loop() -> webSocketEvent(WStype_BIN, ...) -> opusDecoder.write() -> bufferPrint.write()
  virtual size_t write(const uint8_t *buffer, size_t size) override {
    if (webSocket.isConnected()) {
        return _buffer.writeArray(buffer, size);
    }
    return size; //let opusDecoder write, otherwise thread will stuck
  }

private:
  BufferRTOS<uint8_t>& _buffer;
};

BufferPrint bufferPrint(audioBuffer);
OpusAudioDecoder opusDecoder;  //access guarded by wsmutex
BufferRTOS<uint8_t> audioBuffer(AUDIO_BUFFER_SIZE, AUDIO_CHUNK_SIZE);  //producer: networkTask, consumer: audioStreamTask. Thread safe in single producer->single consumer scenario.
I2SStream i2s; //access from audioStreamTask only

// OLD with no pitch shift
VolumeStream volume(i2s); //access from audioStreamTask only
QueueStream<uint8_t> queue(audioBuffer); //access from audioStreamTask only
StreamCopy copier(volume, queue);

// NEW for pitch shift (lossy)
PitchShiftFixedOutput pitchShift(i2s);
VolumeStream volumePitch(pitchShift); //access from audioStreamTask only
StreamCopy pitchCopier(volumePitch, queue);

AudioInfo info(SAMPLE_RATE, CHANNELS, BITS_PER_SAMPLE);
volatile bool i2sOutputFlushScheduled = false;

unsigned long getSpeakingDuration() {
    if (deviceState == SPEAKING && speakingStartTime > 0) {
        return millis() - speakingStartTime;
    }
    return 0;
}

// networkTask -> webSocket.loop() -> webSocketEvent(WStype_TEXT, ...) -> transitionToSpeaking()
void transitionToSpeaking() {
    vTaskDelay(50);

    deviceState = SPEAKING;
    digitalWrite(I2S_SD_OUT, HIGH);
    speakingStartTime = millis();
    
    Serial.println("Transitioned to speaking mode");
}

// networkTask -> transitionToListening()
// ( networkTask -> webSocket.loop() -> webSocketEvent(WStype_TEXT, ...) -> (sets scheduleListeningRestart) -> networkTask -> transitionToListening() )
// Also called directly from webSocketEvent when button is pressed during SPEAKING state
void transitionToListening() {
    deviceState = PROCESSING;   
    scheduleListeningRestart = false;
    Serial.println("Transitioning to listening mode");

    // Flush both input and output streams
    i2sInputFlushScheduled = true;
    i2sOutputFlushScheduled = true;
    
    // Clear the audio buffer to stop any pending audio
    audioBuffer.clear();
    
    // Small delay to allow buffers to clear
    vTaskDelay(50);

    Serial.println("Transitioned to listening mode");

    deviceState = LISTENING;
    digitalWrite(I2S_SD_OUT, LOW);
}

// audioStreamTask -> copier.copy() (conditional on webSocket.isConnected())
void audioStreamTask(void *parameter) {
    Serial.println("Starting I2S stream pipeline...");
    
    pinMode(I2S_SD_OUT, OUTPUT);
    digitalWrite(I2S_SD_OUT, HIGH); // Always enable speaker for concurrent mode

    OpusSettings cfg;
    cfg.sample_rate = SAMPLE_RATE;
    cfg.channels = CHANNELS;
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
    config.sample_rate = SAMPLE_RATE;
    config.channels = CHANNELS;
    config.pin_bck = I2S_BCK_OUT;
    config.pin_ws = I2S_WS_OUT;
    config.pin_data = I2S_DATA_OUT;
    config.port_no = I2S_PORT_OUT;

    config.copyFrom(info);  
    i2s.begin(config);  

    // Initialize both volume streams once
    auto vcfg = volume.defaultConfig();
    vcfg.copyFrom(info);
    vcfg.allow_boost = true;
    volume.begin(vcfg);
    
    auto vcfgPitch = volumePitch.defaultConfig();
    vcfgPitch.copyFrom(info);
    vcfgPitch.allow_boost = true;
    volumePitch.begin(vcfgPitch);

    while (1) {
        if ( i2sOutputFlushScheduled) {
            i2sOutputFlushScheduled = false;
            i2s.flush();
            volume.flush();
            volumePitch.flush();
            queue.flush();
        }

        if (webSocket.isConnected()) {
            // Handle conversation flow
            if (conversationActive) {
                // During conversation, don't play incoming audio
                // Just read from queue to prevent blocking
                queue.read();
                
                // Check for response timeout
                if (waitingForResponse && (millis() - conversationStartTime > responseTimeout)) {
                    Serial.println("⏰ Response timeout - resuming normal audio");
                    waitingForResponse = false;
                    conversationActive = false;
                }
            } else {
                // Normal mode - play incoming audio
                if (currentPitchFactor != 1.0f) {
                    pitchCopier.copy();
                } else {
                    copier.copy();
                }
            }
        }
        else {
            //we should always read from audioBuffer, otherwise writing thread can stuck
            queue.read();
        }
        vTaskDelay(1); 
    }
}


class WebsocketStream : public Print {
public:
    // micTask -> micToWsCopier.copyBytes() -> wsStream.write()
    virtual size_t write(uint8_t b) override {
        if (!webSocket.isConnected()) {
            return 1;
        }
        
        xSemaphoreTake(wsMutex, portMAX_DELAY);
        webSocket.sendBIN(&b, 1);
        xSemaphoreGive(wsMutex);
        return 1;
    }
    
    // micTask -> micToWsCopier.copyBytes() -> wsStream.write()
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        if (size == 0 || !webSocket.isConnected()) {
            return size;
        }
        
        xSemaphoreTake(wsMutex, portMAX_DELAY);
        webSocket.sendBIN(buffer, size);
        xSemaphoreGive(wsMutex);
        return size;
    }
};

WebsocketStream wsStream; //guard with wsMutex
// AUDIO INPUT
I2SStream i2sInput; //access from micTask only
StreamCopy micToWsCopier(wsStream, i2sInput);
volatile bool i2sInputFlushScheduled = false;
const int MIC_COPY_SIZE = 64;

// ECHO CANCELLATION
bool echoCancellationEnabled = true;
int echoCancellationDelay = 50; // milliseconds
float echoCancellationGain = 0.3f; // reduction factor

// Echo cancellation buffer
const int ECHO_BUFFER_SIZE = 2400; // 50ms at 24kHz
int16_t echoBuffer[ECHO_BUFFER_SIZE];
int echoBufferIndex = 0;

// VOICE ACTIVITY DETECTION (VAD)
bool vadEnabled = true;
int vadThreshold = 1000; // Amplitude threshold for voice detection
int vadSilenceDuration = 500; // ms of silence before resuming audio
bool userIsTalking = false;
unsigned long lastVoiceActivity = 0;
int voiceActivityBuffer[64]; // Buffer for VAD analysis

// CONVERSATION FLOW CONTROL
bool conversationActive = false;
bool waitingForResponse = false;
unsigned long conversationStartTime = 0;
unsigned long responseTimeout = 10000; // 10 second timeout for response

void enableEchoCancellation(bool enable) {
    echoCancellationEnabled = enable;
    if (!enable) {
        // Clear echo buffer when disabling
        memset(echoBuffer, 0, sizeof(echoBuffer));
    }
}

void setEchoCancellationDelay(int delay_ms) {
    echoCancellationDelay = delay_ms;
}

void setEchoCancellationGain(float gain) {
    echoCancellationGain = gain;
}

// VOICE ACTIVITY DETECTION FUNCTIONS
void enableVAD(bool enable) {
    vadEnabled = enable;
    if (!enable) {
        userIsTalking = false;
        lastVoiceActivity = 0;
    }
}

void setVADThreshold(int threshold) {
    vadThreshold = threshold;
}

void setVADSilenceDuration(int duration_ms) {
    vadSilenceDuration = duration_ms;
}

bool detectVoiceActivity(int16_t* samples, size_t count) {
    if (!vadEnabled) return false;
    
    // Calculate RMS (Root Mean Square) of the audio samples
    long sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += (long)samples[i] * samples[i];
    }
    
    int rms = sqrt(sum / count);
    
    // Check if RMS exceeds threshold
    if (rms > vadThreshold) {
        lastVoiceActivity = millis();
        return true;
    }
    
    return false;
}

void updateVADStatus() {
    if (!vadEnabled) {
        userIsTalking = false;
        return;
    }
    
    // Check if enough time has passed since last voice activity
    if (millis() - lastVoiceActivity > vadSilenceDuration) {
        userIsTalking = false;
    } else {
        userIsTalking = true;
    }
}

// CONVERSATION FLOW CONTROL FUNCTIONS
void startConversation() {
    conversationActive = true;
    conversationStartTime = millis();
    waitingForResponse = false;
    
    // Flush speaker buffer immediately
    i2sOutputFlushScheduled = true;
    audioBuffer.clear();
    
    Serial.println("🎤 Conversation started - flushing audio buffer");
}

void endConversation() {
    conversationActive = false;
    waitingForResponse = false;
    conversationStartTime = 0;
    
    Serial.println("🔇 Conversation ended");
}

void setResponseTimeout(int timeout_ms) {
    responseTimeout = timeout_ms;
}

bool isWaitingForResponse() {
    return waitingForResponse;
}

// Simple echo cancellation function
void applyEchoCancellation(int16_t* input, int16_t* output, size_t samples) {
    if (!echoCancellationEnabled) {
        memcpy(output, input, samples * sizeof(int16_t));
        return;
    }
    
    for (size_t i = 0; i < samples; i++) {
        // Get the delayed echo sample
        int echoIndex = (echoBufferIndex - echoCancellationDelay * 24 + ECHO_BUFFER_SIZE) % ECHO_BUFFER_SIZE;
        int16_t echoSample = echoBuffer[echoIndex];
        
        // Subtract the echo with gain reduction
        int32_t result = input[i] - (int32_t)(echoSample * echoCancellationGain);
        
        // Clamp to 16-bit range
        if (result > 32767) result = 32767;
        if (result < -32768) result = -32768;
        
        output[i] = (int16_t)result;
        
        // Store current sample in echo buffer
        echoBuffer[echoBufferIndex] = input[i];
        echoBufferIndex = (echoBufferIndex + 1) % ECHO_BUFFER_SIZE;
    }
}

// Custom echo cancellation stream
class EchoCancellationStream : public AudioStream {
public:
    EchoCancellationStream(AudioStream& source) : _source(source) {}
    
    size_t readBytes(uint8_t* buffer, size_t length) override {
        size_t bytesRead = _source.readBytes(buffer, length);
        if (bytesRead > 0 && echoCancellationEnabled) {
            // Convert bytes to samples and apply echo cancellation
            size_t samples = bytesRead / sizeof(int16_t);
            int16_t* samples_ptr = (int16_t*)buffer;
            int16_t* output_ptr = (int16_t*)buffer;
            
            applyEchoCancellation(samples_ptr, output_ptr, samples);
        }
        return bytesRead;
    }
    
    int available() override {
        return _source.available();
    }
    
private:
    AudioStream& _source;
};

void micTask(void *parameter) {
    // Configure and start I2S input stream.
    auto i2sConfig = i2sInput.defaultConfig(RX_MODE);
    i2sConfig.bits_per_sample = BITS_PER_SAMPLE;
    i2sConfig.sample_rate = SAMPLE_RATE;
    i2sConfig.channels = CHANNELS;
    i2sConfig.i2s_format = I2S_LEFT_JUSTIFIED_FORMAT;
    i2sConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    // Configure your I2S input pins appropriately here:
    i2sConfig.pin_bck = I2S_SCK;
    i2sConfig.pin_ws  = I2S_WS;
    i2sConfig.pin_data = I2S_SD;
    i2sConfig.port_no = I2S_PORT_IN;
    i2sInput.begin(i2sConfig);

    // Use echo cancellation stream in concurrent mode
    micToWsCopier.setDelayOnNoData(0);

    // VAD buffer for analysis
    uint8_t vadBuffer[MIC_COPY_SIZE];
    static unsigned long lastVADCheck = 0;
    const int VAD_CHECK_INTERVAL = 50; // Check VAD every 50ms
    static bool wasTalking = false; // Track previous talking state
    static unsigned long silenceStart = 0; // Track silence start time

    while (1) {
        if ( i2sInputFlushScheduled ) {
            i2sInputFlushScheduled = false;
            i2sInput.flush();
        }

        if (webSocket.isConnected()) {
            // Read microphone data for VAD analysis
            size_t bytesRead = i2sInput.readBytes(vadBuffer, MIC_COPY_SIZE);
            
            if (bytesRead > 0) {
                // Check for voice activity periodically
                if (millis() - lastVADCheck > VAD_CHECK_INTERVAL) {
                    lastVADCheck = millis();
                    
                    // Convert bytes to samples for VAD
                    int16_t* samples = (int16_t*)vadBuffer;
                    size_t sampleCount = bytesRead / sizeof(int16_t);
                    
                    // Detect voice activity
                    if (detectVoiceActivity(samples, sampleCount)) {
                        // Voice detected - update status
                        updateVADStatus();
                        
                        // Start conversation if not already active
                        if (!conversationActive && userIsTalking) {
                            startConversation();
                        }
                        
                        // Reset silence timer when voice is detected
                        silenceStart = 0;
                    } else {
                        // No voice detected - update status
                        updateVADStatus();
                        
                        // End conversation if user stopped talking
                        if (conversationActive && !userIsTalking) {
                            // Wait a bit to see if user continues talking
                            if (silenceStart == 0) {
                                silenceStart = millis();
                            } else if (millis() - silenceStart > 1000) { // 1 second of silence
                                endConversation();
                                silenceStart = 0;
                            }
                        }
                    }
                }
                
                // Always send audio to websocket when connected
                micToWsCopier.copyBytes(MIC_COPY_SIZE);
            }
            
            // Yield more frequently
            vTaskDelay(1);
        } else {
            vTaskDelay(10);
        }
    }
}

// WEBSOCKET EVENTS
// networkTask -> webSocket.loop() -> webSocketEvent()
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[WSc] Disconnected!\n");
        deviceState = IDLE;
        break;
    case WStype_CONNECTED:
        Serial.printf("[WSc] Connected to url: %s\n", payload);
        deviceState = CONCURRENT; // Start in concurrent mode when connected
        break;
    case WStype_TEXT:
    {
        Serial.printf("[WSc] get text: %s\n", payload);

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, (char *)payload);

        if (error)
        {
            Serial.println("Error deserializing JSON");
            deviceState = IDLE;
            return;
        }

        String type = doc["type"];

        // auth messages
        if (strcmp((char*)type.c_str(), "auth") == 0) {
            currentVolume = doc["volume_control"].as<int>();
            currentPitchFactor = doc["pitch_factor"].as<float>();

            bool is_ota = doc["is_ota"].as<bool>();
            bool is_reset = doc["is_reset"].as<bool>();

            // Update volumes on both streams
            volume.setVolume(currentVolume / 100.0f);
            volumePitch.setVolume(currentVolume / 100.0f);
            
            // Only initialize pitch shift if needed
            if (currentPitchFactor != 1.0f) {
                auto pcfg = pitchShift.defaultConfig();
                pcfg.copyFrom(info);
                pcfg.pitch_shift = currentPitchFactor;
                pcfg.buffer_size = 512;
                pitchShift.begin(pcfg);
            }

            // Configure echo cancellation if provided
            if (doc["echo_cancellation"].is<bool>()) {
                bool enableEcho = doc["echo_cancellation"].as<bool>();
                enableEchoCancellation(enableEcho);
                Serial.printf("Echo cancellation %s\n", enableEcho ? "enabled" : "disabled");
            }
            
            if (doc["echo_delay"].is<int>()) {
                int delay_ms = doc["echo_delay"].as<int>();
                setEchoCancellationDelay(delay_ms);
                Serial.printf("Echo cancellation delay set to %d ms\n", delay_ms);
            }
            
            if (doc["echo_gain"].is<float>()) {
                float gain = doc["echo_gain"].as<float>();
                setEchoCancellationGain(gain);
                Serial.printf("Echo cancellation gain set to %.2f\n", gain);
            }

            // Configure VAD if provided
            if (doc["vad_enabled"].is<bool>()) {
                bool vadEnabled = doc["vad_enabled"].as<bool>();
                enableVAD(vadEnabled);
                Serial.printf("Voice Activity Detection %s\n", vadEnabled ? "enabled" : "disabled");
            }
            
            if (doc["vad_threshold"].is<int>()) {
                int threshold = doc["vad_threshold"].as<int>();
                setVADThreshold(threshold);
                Serial.printf("VAD threshold set to %d\n", threshold);
            }
            
            if (doc["vad_silence_duration"].is<int>()) {
                int duration = doc["vad_silence_duration"].as<int>();
                setVADSilenceDuration(duration);
                Serial.printf("VAD silence duration set to %d ms\n", duration);
            }

            // Configure conversation flow if provided
            if (doc["response_timeout"].is<int>()) {
                int timeout = doc["response_timeout"].as<int>();
                setResponseTimeout(timeout);
                Serial.printf("Response timeout set to %d ms\n", timeout);
            }

            if (is_ota) {
                Serial.println("OTA update received");
                setOTAStatusInNVS(OTA_IN_PROGRESS);
                ESP.restart();
            }

            if (is_reset) {
                Serial.println("Factory reset received");
                // setFactoryResetStatusInNVS(true);
                ESP.restart();
            }
        }

        // oai messages
        if (strcmp((char*)type.c_str(), "server") == 0) {
            String msg = doc["msg"];
            Serial.println(msg);

            if (strcmp((char*)msg.c_str(), "RESPONSE.COMPLETE") == 0 || strcmp((char*)msg.c_str(), "RESPONSE.ERROR") == 0) {
                Serial.println("Received RESPONSE.COMPLETE or RESPONSE.ERROR");

                // Check if volume_control is included in the message
                if (doc["volume_control"].is<int>()) {
                    int newVolume = doc["volume_control"].as<int>();
                    volume.setVolume(newVolume / 100.0f);
                }

                // End conversation and resume normal audio
                if (conversationActive) {
                    endConversation();
                }
            } else if (strcmp((char*)msg.c_str(), "AUDIO.COMMITTED") == 0) {
                deviceState = PROCESSING; 
            } else if (strcmp((char*)msg.c_str(), "RESPONSE.CREATED") == 0) {
                Serial.println("Received RESPONSE.CREATED, transitioning to speaking");
                
                // If in conversation mode, start waiting for response
                if (conversationActive) {
                    waitingForResponse = true;
                    Serial.println("🎧 Waiting for response from server...");
                }
                
                transitionToSpeaking();
            } else if (strcmp((char*)msg.c_str(), "CLEAR_BUFFER") == 0) {
                Serial.println("Received CLEAR_BUFFER, clearing audio buffer and stopping playback");
                // Clear the audio buffer to stop any pending audio
                audioBuffer.clear();
                // Flush the I2S output to immediately stop speaker audio
                i2sOutputFlushScheduled = true;
                // Also flush volume streams immediately for instant audio stop
                i2s.flush();
                volume.flush();
                queue.flush();
                
            } else if (strcmp((char*)msg.c_str(), "SESSION.END") == 0) {
                Serial.println("Received SESSION.END, going to sleep");
                sleepRequested = true;
            }
        }
    }
        break;
    case WStype_BIN:
    {
        if (scheduleListeningRestart) {
            Serial.println("Skipping audio data due to button/touch interrupt.");
            // In concurrent mode, we don't need to transition states
            // Just continue processing audio
            break;
        }
        
        // Process audio data regardless of state in concurrent mode
        size_t processed = opusDecoder.write(payload, length);
        if (processed != length) {
            Serial.printf("Warning: Only processed %d/%d bytes\n", processed, length);
        }
        break;
      }
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

// wifiTask -> WIFIMANAGER::loop() -> WIFIMANAGER::tryConnect() -> connectCb() -> websocketSetup()
void websocketSetup(String server_domain, int port, String path)
{
    String headers = "Authorization: Bearer " + String(authTokenGlobal);

    xSemaphoreTake(wsMutex, portMAX_DELAY);

    webSocket.setExtraHeaders(headers.c_str());
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(1000);
    webSocket.disableHeartbeat();

    // webSocket.enableHeartbeat(30000, 15000, 3); // 30s ping interval, 15s timeout, 3 retries

    #ifdef DEV_MODE
    webSocket.begin(server_domain.c_str(), port, path.c_str());
    #else
    webSocket.beginSslWithCA(server_domain.c_str(), port, path.c_str(), CA_cert);
    #endif

    xSemaphoreGive(wsMutex);
}

// networkTask -> webSocket.loop()
void networkTask(void *parameter) {
    while (1) {
        xSemaphoreTake(wsMutex, portMAX_DELAY);

        // Check to see if a transition to listening mode is scheduled.
        if (scheduleListeningRestart && millis() >= scheduledTime) {
            transitionToListening();
        }

        webSocket.loop();
        xSemaphoreGive(wsMutex);

        vTaskDelay(1);
    }
}

// Helper: find idle task names for both cores
// Idle-hook based CPU monitor (no FreeRTOS runtime stats needed)
static volatile uint64_t idleCountCore0 = 0;
static volatile uint64_t idleCountCore1 = 0;

extern "C" bool idleHookCore0() {
    idleCountCore0++;
    return true;
}

extern "C" bool idleHookCore1() {
    idleCountCore1++;
    return true;
}

void cpuMonitorTask(void *parameter) {
    static bool hooksRegistered = false;
    if (!hooksRegistered) {
        if (!esp_register_freertos_idle_hook_for_cpu(idleHookCore0, 0)) {
            Serial.println("CPU Monitor: registered idle hook for core 0");
        } else {
            Serial.println("CPU Monitor: failed to register idle hook for core 0");
        }
        if (!esp_register_freertos_idle_hook_for_cpu(idleHookCore1, 1)) {
            Serial.println("CPU Monitor: registered idle hook for core 1");
        } else {
            Serial.println("CPU Monitor: failed to register idle hook for core 1");
        }
        hooksRegistered = true;
    }

    Serial.println("CPU Monitor (idle-hook) started (5s interval)");

    uint64_t lastIdle0 = idleCountCore0;
    uint64_t lastIdle1 = idleCountCore1;
    uint64_t maxIdleDelta0 = 0;
    uint64_t maxIdleDelta1 = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint64_t nowIdle0 = idleCountCore0;
        uint64_t nowIdle1 = idleCountCore1;
        uint64_t d0 = (nowIdle0 - lastIdle0);
        uint64_t d1 = (nowIdle1 - lastIdle1);
        lastIdle0 = nowIdle0;
        lastIdle1 = nowIdle1;

        if (d0 > maxIdleDelta0) maxIdleDelta0 = d0;
        if (d1 > maxIdleDelta1) maxIdleDelta1 = d1;

        float util0 = (maxIdleDelta0 > 0) ? (100.0f * (1.0f - ((float)d0 / (float)maxIdleDelta0))) : 0.0f;
        float util1 = (maxIdleDelta1 > 0) ? (100.0f * (1.0f - ((float)d1 / (float)maxIdleDelta1))) : 0.0f;

        if (util0 < 0) util0 = 0; if (util0 > 100) util0 = 100;
        if (util1 < 0) util1 = 0; if (util1 > 100) util1 = 100;

        Serial.printf("Core0 Util: %.1f%% | Core1 Util: %.1f%%\n", util0, util1);
    }
}