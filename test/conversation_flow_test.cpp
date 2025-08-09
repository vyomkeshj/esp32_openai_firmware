#include "Arduino.h"
#include "Audio.h"
#include "Config.h"
#include "AudioTools.h"

// Conversation Flow Test Configuration
#define TEST_DURATION_MS 60000  // 60 seconds
#define CONVERSATION_CHECK_INTERVAL 100  // Check every 100ms
#define AUDIO_CHUNK_SIZE 512

// Test state
bool testRunning = false;
unsigned long testStartTime = 0;
int conversationsStarted = 0;
int conversationsEnded = 0;
int responsesReceived = 0;

// Audio streams for testing
I2SStream i2sInput;
I2SStream i2sOutput;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Conversation Flow Test ===");
    Serial.println("This test verifies the request-response conversation flow");
    
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
    i2sConfigOut.pin_data = I2S_DATA_OUT;
    i2sConfigOut.port_no = I2S_PORT_OUT;
    i2sOutput.begin(i2sConfigOut);
    
    // Enable speaker
    pinMode(I2S_SD_OUT, OUTPUT);
    digitalWrite(I2S_SD_OUT, HIGH);
    
    // Configure VAD and conversation flow
    enableVAD(true);
    setVADThreshold(800);
    setVADSilenceDuration(300);
    setResponseTimeout(15000); // 15 second timeout
    
    Serial.println("Conversation flow configured:");
    Serial.printf("  VAD Threshold: %d\n", vadThreshold);
    Serial.printf("  Silence Duration: %d ms\n", vadSilenceDuration);
    Serial.printf("  Response Timeout: %d ms\n", responseTimeout);
    Serial.println("Starting conversation flow test...");
    Serial.println("Speak into the microphone to start conversations");
    
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
        Serial.println("=== Conversation Flow Test Complete ===");
        Serial.printf("Conversations started: %d\n", conversationsStarted);
        Serial.printf("Conversations ended: %d\n", conversationsEnded);
        Serial.printf("Responses received: %d\n", responsesReceived);
        Serial.printf("Total test time: %d ms\n", millis() - testStartTime);
        
        // Disable speaker
        digitalWrite(I2S_SD_OUT, LOW);
        
        Serial.println("Test finished. Check serial output for results.");
        return;
    }
    
    // Test conversation flow functionality
    testConversationFlow();
    
    // Small delay
    vTaskDelay(10);
}

void testConversationFlow() {
    static uint8_t audioBuffer[AUDIO_CHUNK_SIZE];
    static unsigned long lastCheck = 0;
    static bool lastConversationActive = false;
    static bool lastWaitingForResponse = false;
    
    // Check conversation flow periodically
    if (millis() - lastCheck > CONVERSATION_CHECK_INTERVAL) {
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
            
            // Track conversation state changes
            if (conversationActive && !lastConversationActive) {
                conversationsStarted++;
                Serial.println("🎤 CONVERSATION STARTED");
            } else if (!conversationActive && lastConversationActive) {
                conversationsEnded++;
                Serial.println("🔇 CONVERSATION ENDED");
            }
            
            // Track response waiting
            if (waitingForResponse && !lastWaitingForResponse) {
                Serial.println("🎧 WAITING FOR RESPONSE");
            } else if (!waitingForResponse && lastWaitingForResponse) {
                responsesReceived++;
                Serial.println("✅ RESPONSE RECEIVED");
            }
            
            lastConversationActive = conversationActive;
            lastWaitingForResponse = waitingForResponse;
            
            // Print status periodically
            static unsigned long lastStatusPrint = 0;
            if (millis() - lastStatusPrint > 3000) { // Every 3 seconds
                lastStatusPrint = millis();
                Serial.printf("Status: %s | %s | RMS: %d\n", 
                           conversationActive ? "CONVERSATION" : "NORMAL",
                           waitingForResponse ? "WAITING" : "READY",
                           calculateRMS(samples, sampleCount));
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

// Simulate conversation flow for testing
void simulateConversationFlow() {
    Serial.println("Simulating conversation flow...");
    
    // Simulate user starting to talk
    Serial.println("1. User starts talking...");
    startConversation();
    delay(2000);
    
    // Simulate waiting for response
    Serial.println("2. Waiting for response...");
    waitingForResponse = true;
    delay(3000);
    
    // Simulate response received
    Serial.println("3. Response received...");
    waitingForResponse = false;
    delay(2000);
    
    // Simulate conversation end
    Serial.println("4. Conversation ended...");
    endConversation();
    
    Serial.println("Conversation flow simulation complete");
}

// Test different conversation scenarios
void testConversationScenarios() {
    Serial.println("Testing different conversation scenarios...");
    
    // Scenario 1: Quick response
    Serial.println("Scenario 1: Quick response");
    startConversation();
    delay(1000);
    waitingForResponse = true;
    delay(2000);
    waitingForResponse = false;
    endConversation();
    delay(1000);
    
    // Scenario 2: Slow response
    Serial.println("Scenario 2: Slow response");
    startConversation();
    delay(1000);
    waitingForResponse = true;
    delay(8000);
    waitingForResponse = false;
    endConversation();
    delay(1000);
    
    // Scenario 3: Timeout
    Serial.println("Scenario 3: Response timeout");
    startConversation();
    delay(1000);
    waitingForResponse = true;
    delay(16000); // Longer than timeout
    endConversation();
    delay(1000);
    
    Serial.println("Conversation scenarios complete");
}
