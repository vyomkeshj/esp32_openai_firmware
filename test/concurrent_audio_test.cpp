#include "Arduino.h"
#include "Audio.h"
#include "Config.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"
#include <WebSocketsClient.h>

// Test configuration
#define TEST_DURATION_MS 10000  // 10 seconds
#define AUDIO_CHUNK_SIZE 512
#define SAMPLE_RATE 24000
#define CHANNELS 1
#define BITS_PER_SAMPLE 16

// Test state
bool testRunning = false;
unsigned long testStartTime = 0;
int audioChunksProcessed = 0;
int micChunksProcessed = 0;
int speakerChunksProcessed = 0;

// Audio streams for testing
I2SStream i2sInput;
I2SStream i2sOutput;
BufferRTOS<uint8_t> testAudioBuffer(1024 * 10, 1024);

// Echo cancellation test
bool echoTestEnabled = true;
int16_t echoTestBuffer[2400]; // 100ms at 24kHz
int echoTestIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Concurrent Audio Test ===");
    Serial.println("This test verifies that microphone and speaker can work simultaneously");
    
    // Configure I2S input
    auto i2sConfigIn = i2sInput.defaultConfig(RX_MODE);
    i2sConfigIn.bits_per_sample = BITS_PER_SAMPLE;
    i2sConfigIn.sample_rate = SAMPLE_RATE;
    i2sConfigIn.channels = CHANNELS;
    i2sConfigIn.i2s_format = I2S_LEFT_JUSTIFIED_FORMAT;
    i2sConfigIn.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2sConfigIn.pin_bck = I2S_SCK;
    i2sConfigIn.pin_ws = I2S_WS;
    i2sConfigIn.pin_data = I2S_SD;
    i2sConfigIn.port_no = I2S_PORT_IN;
    i2sInput.begin(i2sConfigIn);
    
    // Configure I2S output
    auto i2sConfigOut = i2sOutput.defaultConfig(TX_MODE);
    i2sConfigOut.bits_per_sample = BITS_PER_SAMPLE;
    i2sConfigOut.sample_rate = SAMPLE_RATE;
    i2sConfigOut.channels = CHANNELS;
    i2sConfigOut.pin_bck = I2S_BCK_OUT;
    i2sConfigOut.pin_ws = I2S_WS_OUT;
    i2sConfigOut.pin_data = I2S_DATA_OUT;
    i2sConfigOut.port_no = I2S_PORT_OUT;
    i2sOutput.begin(i2sConfigOut);
    
    // Enable speaker
    pinMode(I2S_SD_OUT, OUTPUT);
    digitalWrite(I2S_SD_OUT, HIGH);
    
    Serial.println("I2S streams configured");
    Serial.println("Starting concurrent audio test...");
    
    testRunning = true;
    testStartTime = millis();
}

void loop() {
    if (!testRunning) {
        return;
    }
    
    // Check if test duration has elapsed
    if (millis() - testStartTime > TEST_DURATION_MS) {
        testRunning = false;
        Serial.println("=== Test Complete ===");
        Serial.printf("Audio chunks processed: %d\n", audioChunksProcessed);
        Serial.printf("Microphone chunks processed: %d\n", micChunksProcessed);
        Serial.printf("Speaker chunks processed: %d\n", speakerChunksProcessed);
        Serial.printf("Total test time: %d ms\n", millis() - testStartTime);
        
        // Disable speaker
        digitalWrite(I2S_SD_OUT, LOW);
        
        Serial.println("Test finished. Check serial output for results.");
        return;
    }
    
    // Simulate concurrent audio processing
    processMicrophoneInput();
    processSpeakerOutput();
    
    // Small delay to prevent overwhelming the system
    vTaskDelay(1);
}

void processMicrophoneInput() {
    static uint8_t micBuffer[AUDIO_CHUNK_SIZE];
    static int micBufferIndex = 0;
    
    // Read from microphone
    size_t bytesRead = i2sInput.readBytes(micBuffer + micBufferIndex, AUDIO_CHUNK_SIZE - micBufferIndex);
    micBufferIndex += bytesRead;
    
    if (micBufferIndex >= AUDIO_CHUNK_SIZE) {
        // Process a full chunk
        if (echoTestEnabled) {
            applyEchoCancellationTest(micBuffer, AUDIO_CHUNK_SIZE / 2);
        }
        
        // Simulate sending to websocket (just count for test)
        micChunksProcessed++;
        
        // Reset buffer
        micBufferIndex = 0;
        
        if (micChunksProcessed % 100 == 0) {
            Serial.printf("Mic chunks: %d\n", micChunksProcessed);
        }
    }
}

void processSpeakerOutput() {
    static uint8_t speakerBuffer[AUDIO_CHUNK_SIZE];
    static int speakerBufferIndex = 0;
    
    // Generate test audio (sine wave)
    generateTestAudio(speakerBuffer, AUDIO_CHUNK_SIZE);
    
    // Write to speaker
    size_t bytesWritten = i2sOutput.writeBytes(speakerBuffer, AUDIO_CHUNK_SIZE);
    
    if (bytesWritten > 0) {
        speakerChunksProcessed++;
        
        if (speakerChunksProcessed % 100 == 0) {
            Serial.printf("Speaker chunks: %d\n", speakerChunksProcessed);
        }
    }
}

void generateTestAudio(uint8_t* buffer, size_t size) {
    static float phase = 0.0f;
    const float frequency = 440.0f; // A4 note
    const float amplitude = 0.3f;
    
    int16_t* samples = (int16_t*)buffer;
    size_t numSamples = size / sizeof(int16_t);
    
    for (size_t i = 0; i < numSamples; i++) {
        float sample = amplitude * sin(phase);
        samples[i] = (int16_t)(sample * 32767.0f);
        
        phase += 2.0f * PI * frequency / SAMPLE_RATE;
        if (phase > 2.0f * PI) {
            phase -= 2.0f * PI;
        }
    }
}

void applyEchoCancellationTest(uint8_t* buffer, size_t samples) {
    int16_t* samples_ptr = (int16_t*)buffer;
    
    for (size_t i = 0; i < samples; i++) {
        // Get the delayed echo sample
        int echoIndex = (echoTestIndex - 1200 + 2400) % 2400; // 50ms delay
        int16_t echoSample = echoTestBuffer[echoIndex];
        
        // Subtract the echo with gain reduction
        int32_t result = samples_ptr[i] - (int32_t)(echoSample * 0.3f);
        
        // Clamp to 16-bit range
        if (result > 32767) result = 32767;
        if (result < -32768) result = -32768;
        
        samples_ptr[i] = (int16_t)result;
        
        // Store current sample in echo buffer
        echoTestBuffer[echoTestIndex] = samples_ptr[i];
        echoTestIndex = (echoTestIndex + 1) % 2400;
    }
}

// Test echo cancellation configuration
void testEchoCancellationConfig() {
    Serial.println("Testing echo cancellation configuration...");
    
    enableEchoCancellation(true);
    setEchoCancellationDelay(50);
    setEchoCancellationGain(0.3f);
    
    Serial.println("Echo cancellation enabled with 50ms delay and 0.3 gain");
}

// Test volume control
void testVolumeControl() {
    Serial.println("Testing volume control...");
    
    // Test different volume levels
    for (int vol = 10; vol <= 100; vol += 30) {
        volume.setVolume(vol / 100.0f);
        Serial.printf("Volume set to %d%%\n", vol);
        delay(1000);
    }
    
    // Reset to default
    volume.setVolume(0.7f);
    Serial.println("Volume reset to 70%");
}
