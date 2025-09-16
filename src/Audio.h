
#include "Print.h"
#include "Config.h"
#include "AudioTools.h"
// #include "AudioTools/Concurrency/RTOS.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"
#include <WebSocketsClient.h>
#include <ESP32-SpeexDSP.h>

extern SemaphoreHandle_t wsMutex;
extern WebSocketsClient webSocket;

extern TaskHandle_t speakerTaskHandle;
extern TaskHandle_t micTaskHandle;
extern TaskHandle_t networkTaskHandle;
extern TaskHandle_t vadTaskHandle;

extern volatile bool scheduleListeningRestart;
extern unsigned long scheduledTime;
extern unsigned long speakingStartTime;

extern int currentVolume;
extern const int CHANNELS;         // Mono
extern const int BITS_PER_SAMPLE; // 16-bit audio

// AUDIO OUTPUT
constexpr size_t AUDIO_BUFFER_SIZE = 1024 * 10;     // total bytes in the buffer
constexpr size_t AUDIO_CHUNK_SIZE  = 1024;         // ideal read/write chunk size
extern OpusAudioDecoder opusDecoder;
extern BufferRTOS<uint8_t> audioBuffer;
extern I2SStream i2s; 
extern VolumeStream volume;
extern QueueStream<uint8_t> queue;
extern StreamCopy copier;

// NEW for pitch shift
extern VolumeStream volumePitch;
extern StreamCopy pitchCopier;

extern AudioInfo info;
extern volatile bool i2sOutputFlushScheduled;

// AUDIO INPUT
extern I2SStream i2sInput;
extern StreamCopy micToWsCopier;
extern volatile bool i2sInputFlushScheduled;

// ECHO CANCELLATION
extern bool echoCancellationEnabled;
extern int echoCancellationDelay;
extern float echoCancellationGain;

// VOICE ACTIVITY DETECTION (VAD)
extern bool vadEnabled;
extern ESP32SpeexDSP dsp;

// CONVERSATION FLOW CONTROL
extern bool conversationActive;
extern bool waitingForResponse;
extern unsigned long conversationStartTime;
extern unsigned long responseTimeout;

// WEBSOCKET
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void websocketSetup(String server_domain, int port, String path);
void networkTask(void *parameter);

// AUDIO OUTPUT
unsigned long getSpeakingDuration();
void audioStreamTask(void *parameter);

// AUDIO INPUT
void micTask(void *parameter);

// VAD TASK
void vadTask(void *parameter);

// ECHO CANCELLATION
void enableEchoCancellation(bool enable);
void setEchoCancellationDelay(int delay_ms);
void setEchoCancellationGain(float gain);

// VOICE ACTIVITY DETECTION
// VAD is now handled by ESP32-SpeexDSP library

// CONVERSATION FLOW CONTROL
void startConversation();
void endConversation();
void setResponseTimeout(int timeout_ms);
bool isWaitingForResponse();