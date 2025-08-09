#include "Arduino.h"
#include "Audio.h"
#include "Config.h"
#include "AudioTools.h"

// VAD Test Configuration
#define TEST_DURATION_MS 30000  // 30 seconds
#define VAD_CHECK_INTERVAL 100  // Check VAD every 100ms
#define AUDIO_CHUNK_SIZE 512

// Test state
bool testRunning = false;
unsigned long testStartTime = 0;
int voiceDetections = 0;
int silenceDetections = 0;
unsigned long lastVADStatus = 0;

// Audio streams for testing
I2SStream i2sInput;
I2SStream i2sOutput;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Voice Activity Detection Test ===");
    Serial.println("This test verifies VAD functionality and audio pausing");
    
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
    
    // Configure VAD
    enableVAD(true);
    setVADThreshold(800);  // Lower threshold for testing
    setVADSilenceDuration(300);  // 300ms silence duration
    
    Serial.println("VAD configured:");
    Serial.printf("  Threshold: %d\n", vadThreshold);
    Serial.printf("  Silence Duration: %d ms\n", vadSilenceDuration);
    Serial.println("Starting VAD test...");
    Serial.println("Speak into the microphone to test VAD detection");
    
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
        Serial.println("=== VAD Test Complete ===");
        Serial.printf("Voice detections: %d\n", voiceDetections);
        Serial.printf("Silence detections: %d\n", silenceDetections);
        Serial.printf("Total test time: %d ms\n", millis() - testStartTime);
        
        // Disable speaker
        digitalWrite(I2S_SD_OUT, LOW);
        
        Serial.println("Test finished. Check serial output for results.");
        return;
    }
    
    // Test VAD functionality
    testVADDetection();
    
    // Small delay
    vTaskDelay(10);
}

void testVADDetection() {
    static uint8_t audioBuffer[AUDIO_CHUNK_SIZE];
    static unsigned long lastCheck = 0;
    
    // Check VAD periodically
    if (millis() - lastCheck > VAD_CHECK_INTERVAL) {
        lastCheck = millis();
        
        // Read microphone data
        size_t bytesRead = i2sInput.readBytes(audioBuffer, AUDIO_CHUNK_SIZE);
        
        if (bytesRead > 0) {
            // Convert to samples for VAD
            int16_t* samples = (int16_t*)audioBuffer;
            size_t sampleCount = bytesRead / sizeof(int16_t);
            
            // Detect voice activity
            bool voiceDetected = detectVoiceActivity(samples, sampleCount);
            
            // Update VAD status
            updateVADStatus();
            
            // Track detections
            static bool lastUserTalking = false;
            if (userIsTalking && !lastUserTalking) {
                voiceDetections++;
                Serial.println("🎤 VOICE DETECTED - Audio paused");
            } else if (!userIsTalking && lastUserTalking) {
                silenceDetections++;
                Serial.println("🔇 SILENCE DETECTED - Audio resumed");
            }
            lastUserTalking = userIsTalking;
            
            // Print VAD status periodically
            static unsigned long lastStatusPrint = 0;
            if (millis() - lastStatusPrint > 2000) { // Every 2 seconds
                lastStatusPrint = millis();
                Serial.printf("VAD Status: %s (RMS: %d, Threshold: %d)\n", 
                           userIsTalking ? "TALKING" : "SILENT",
                           calculateRMS(samples, sampleCount),
                           vadThreshold);
            }
        }
    }
}

int calculateRMS(int16_t* samples, size_t count) {
    long sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += (long)samples[i] * samples[i];
    }
    return sqrt(sum / count);
}

// Test different VAD configurations
void testVADConfigurations() {
    Serial.println("Testing different VAD configurations...");
    
    // Test 1: High sensitivity
    Serial.println("Test 1: High sensitivity (low threshold)");
    setVADThreshold(500);
    setVADSilenceDuration(200);
    delay(5000);
    
    // Test 2: Low sensitivity
    Serial.println("Test 2: Low sensitivity (high threshold)");
    setVADThreshold(2000);
    setVADSilenceDuration(1000);
    delay(5000);
    
    // Test 3: Medium sensitivity
    Serial.println("Test 3: Medium sensitivity");
    setVADThreshold(1000);
    setVADSilenceDuration(500);
    delay(5000);
    
    Serial.println("VAD configuration tests complete");
}

// Test VAD with different audio levels
void testVADWithAudioLevels() {
    Serial.println("Testing VAD with different audio levels...");
    
    // Generate test tones at different volumes
    for (int volume = 10; volume <= 100; volume += 30) {
        Serial.printf("Testing with volume level: %d%%\n", volume);
        generateTestTone(volume);
        delay(3000);
    }
}

void generateTestTone(int volumePercent) {
    const int duration = 2000; // 2 seconds
    const float frequency = 440.0f; // A4 note
    const float amplitude = volumePercent / 100.0f;
    
    unsigned long startTime = millis();
    float phase = 0.0f;
    
    while (millis() - startTime < duration) {
        // Generate sine wave
        float sample = amplitude * sin(phase);
        int16_t audioSample = (int16_t)(sample * 32767.0f);
        
        // Write to speaker
        i2sOutput.writeBytes(&audioSample, sizeof(int16_t));
        
        // Update phase
        phase += 2.0f * PI * frequency / SAMPLE_RATE;
        if (phase > 2.0f * PI) {
            phase -= 2.0f * PI;
        }
        
        vTaskDelay(1);
    }
}
