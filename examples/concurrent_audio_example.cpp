#include "Arduino.h"
#include "Audio.h"
#include "Config.h"
#include "WifiManager.h"

// Example: Real-time call with concurrent audio
// This example demonstrates how to use the concurrent audio system
// for a real-time call experience

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Concurrent Audio Example ===");
    Serial.println("This example shows how to use concurrent audio for real-time calls");
    
    // Initialize device metadata
    setupDeviceMetadata();
    
    // Create mutex for websocket
    wsMutex = xSemaphoreCreateMutex();
    
    // Create audio tasks
    xTaskCreatePinnedToCore(
        audioStreamTask,   // Function
        "Speaker Task",    // Name
        4096,              // Stack size
        NULL,              // Parameters
        3,                 // Priority
        NULL,              // Handle
        1                  // Core 1 (application core)
    );

    xTaskCreatePinnedToCore(
        micTask,           // Function
        "Microphone Task", // Name
        4096,              // Stack size
        NULL,              // Parameters
        4,                 // Priority
        NULL,              // Handle
        1                  // Core 1 (application core)
    );

    xTaskCreatePinnedToCore(
        networkTask,       // Function
        "Websocket Task",  // Name
        8192,              // Stack size
        NULL,              // Parameters
        configMAX_PRIORITIES-1, // Highest priority
        NULL,              // Handle
        0                  // Core 0 (protocol core)
    );

    // Setup WiFi and WebSocket
    setupWiFi();
    
    Serial.println("Concurrent audio system initialized");
    Serial.println("Both microphone and speaker are now active");
    Serial.println("Audio will be sent and received simultaneously");
}

void loop() {
    // Main loop - most work is done in tasks
    vTaskDelay(1000);
    
    // Print status every second
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 1000) {
        lastStatus = millis();
        
        Serial.printf("Device State: %d\n", deviceState);
        Serial.printf("WebSocket Connected: %s\n", webSocket.isConnected() ? "Yes" : "No");
        Serial.printf("Echo Cancellation: %s\n", echoCancellationEnabled ? "Enabled" : "Disabled");
        Serial.printf("Volume: %d%%\n", currentVolume);
        Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    }
}

// Example: Configure echo cancellation for different environments
void configureForSmallRoom() {
    Serial.println("Configuring for small room...");
    enableEchoCancellation(true);
    setEchoCancellationDelay(30);  // Shorter delay for small room
    setEchoCancellationGain(0.5f); // Higher gain reduction
}

void configureForLargeRoom() {
    Serial.println("Configuring for large room...");
    enableEchoCancellation(true);
    setEchoCancellationDelay(80);  // Longer delay for large room
    setEchoCancellationGain(0.2f); // Lower gain reduction
}

void configureForHeadphones() {
    Serial.println("Configuring for headphones...");
    enableEchoCancellation(false);  // No echo cancellation needed
}

// Example: Dynamic volume control
void adjustVolumeForEnvironment() {
    // Simulate environment detection
    int ambientNoise = analogRead(34); // Example: noise sensor
    
    if (ambientNoise > 2000) {
        // Loud environment
        currentVolume = 90;
        volume.setVolume(0.9f);
        volumePitch.setVolume(0.9f);
        Serial.println("Volume increased for loud environment");
    } else if (ambientNoise < 500) {
        // Quiet environment
        currentVolume = 50;
        volume.setVolume(0.5f);
        volumePitch.setVolume(0.5f);
        Serial.println("Volume decreased for quiet environment");
    }
}

// Example: Audio quality monitoring
void monitorAudioQuality() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 5000) { // Check every 5 seconds
        lastCheck = millis();
        
        // Check buffer usage
        size_t availableSpace = audioBuffer.availableForWrite();
        size_t totalSpace = audioBuffer.size();
        float bufferUsage = (float)(totalSpace - availableSpace) / totalSpace * 100.0f;
        
        Serial.printf("Audio Buffer Usage: %.1f%%\n", bufferUsage);
        
        if (bufferUsage > 80.0f) {
            Serial.println("Warning: High buffer usage detected");
        }
        
        if (bufferUsage < 10.0f) {
            Serial.println("Warning: Low buffer usage - possible audio dropouts");
        }
    }
}

// Example: Handle button press for volume control
void handleVolumeButton() {
    static unsigned long lastPress = 0;
    static int volumeStep = 0;
    
    // Simulate button press (replace with actual button logic)
    bool buttonPressed = digitalRead(2) == LOW;
    
    if (buttonPressed && millis() - lastPress > 200) {
        lastPress = millis();
        
        // Cycle through volume levels
        volumeStep = (volumeStep + 1) % 4;
        int newVolume = 25 + (volumeStep * 25); // 25%, 50%, 75%, 100%
        
        currentVolume = newVolume;
        volume.setVolume(newVolume / 100.0f);
        volumePitch.setVolume(newVolume / 100.0f);
        
        Serial.printf("Volume set to %d%%\n", newVolume);
    }
}

// Example: WebSocket message handler for concurrent mode
void handleConcurrentAudioMessage(JsonDocument& doc) {
    String type = doc["type"];
    
    if (type == "concurrent_config") {
        // Handle concurrent audio configuration
        if (doc.containsKey("echo_enabled")) {
            enableEchoCancellation(doc["echo_enabled"].as<bool>());
        }
        
        if (doc.containsKey("echo_delay")) {
            setEchoCancellationDelay(doc["echo_delay"].as<int>());
        }
        
        if (doc.containsKey("echo_gain")) {
            setEchoCancellationGain(doc["echo_gain"].as<float>());
        }
        
        if (doc.containsKey("volume")) {
            int vol = doc["volume"].as<int>();
            currentVolume = vol;
            volume.setVolume(vol / 100.0f);
            volumePitch.setVolume(vol / 100.0f);
        }
        
        Serial.println("Concurrent audio configuration updated");
    }
}

// Example: Performance monitoring
void monitorPerformance() {
    static unsigned long lastMonitor = 0;
    if (millis() - lastMonitor > 10000) { // Monitor every 10 seconds
        lastMonitor = millis();
        
        // Get task statistics
        UBaseType_t speakerPriority = uxTaskPriorityGet(speakerTaskHandle);
        UBaseType_t micPriority = uxTaskPriorityGet(micTaskHandle);
        UBaseType_t networkPriority = uxTaskPriorityGet(networkTaskHandle);
        
        Serial.println("=== Performance Monitor ===");
        Serial.printf("Speaker Task Priority: %d\n", speakerPriority);
        Serial.printf("Microphone Task Priority: %d\n", micPriority);
        Serial.printf("Network Task Priority: %d\n", networkPriority);
        Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
        Serial.println("==========================");
    }
}
