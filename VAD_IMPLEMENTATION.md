# Voice Activity Detection (VAD) Implementation

## Overview

The Voice Activity Detection (VAD) system automatically pauses incoming audio when the user starts talking, preventing feedback and providing a more natural conversation experience.

## How It Works

### 1. **Voice Detection**
- **RMS Analysis**: Calculates Root Mean Square of audio samples
- **Threshold Comparison**: Compares RMS against configurable threshold
- **Real-time Processing**: Analyzes microphone input every 50ms

### 2. **Audio Pausing**
- **Incoming Audio**: Pauses WebSocket audio when user is talking
- **Outgoing Audio**: Continues sending microphone data to server
- **Seamless Transition**: Resumes audio after silence detection

### 3. **Configurable Parameters**
- **Threshold**: Audio amplitude level for voice detection
- **Silence Duration**: Time to wait before resuming audio
- **Enable/Disable**: Turn VAD on/off via WebSocket

## Implementation Details

### VAD Configuration
```cpp
// VAD Parameters
bool vadEnabled = true;
int vadThreshold = 1000;        // Amplitude threshold
int vadSilenceDuration = 500;   // ms of silence before resuming
bool userIsTalking = false;     // Current VAD status
```

### Voice Detection Algorithm
```cpp
bool detectVoiceActivity(int16_t* samples, size_t count) {
    // Calculate RMS (Root Mean Square)
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
```

### Audio Stream Control
```cpp
// In audioStreamTask()
if (webSocket.isConnected()) {
    // Only play incoming audio if user is not talking
    if (!userIsTalking) {
        copier.copy(); // Play incoming audio
    } else {
        queue.read();  // Skip audio to prevent feedback
    }
}
```

## WebSocket Configuration

### VAD Configuration Message
```json
{
    "type": "auth",
    "vad_enabled": true,
    "vad_threshold": 1000,
    "vad_silence_duration": 500
}
```

### Parameters
- **`vad_enabled`**: Enable/disable VAD (boolean)
- **`vad_threshold`**: Audio amplitude threshold (int)
- **`vad_silence_duration`**: Silence duration in ms (int)

## Usage Examples

### 1. **Basic VAD Usage**
```cpp
// Enable VAD with default settings
enableVAD(true);
setVADThreshold(1000);
setVADSilenceDuration(500);
```

### 2. **Sensitive VAD (for quiet environments)**
```cpp
// Lower threshold for quiet environments
setVADThreshold(500);
setVADSilenceDuration(300);
```

### 3. **Less Sensitive VAD (for noisy environments)**
```cpp
// Higher threshold for noisy environments
setVADThreshold(2000);
setVADSilenceDuration(800);
```

### 4. **Disable VAD**
```cpp
// Turn off VAD completely
enableVAD(false);
```

## Testing

### VAD Test File
Use `test/vad_test.cpp` to test VAD functionality:

1. **Upload the test file**
2. **Monitor serial output** for VAD status
3. **Speak into microphone** to test detection
4. **Check detection counts** and timing

### Test Features
- **Real-time VAD monitoring**
- **Detection statistics**
- **Configuration testing**
- **Audio level testing**

## Performance Characteristics

### **CPU Usage**
- **VAD Analysis**: ~2-5% CPU
- **RMS Calculation**: Minimal overhead
- **Status Updates**: Negligible impact

### **Latency**
- **Voice Detection**: ~50ms response time
- **Audio Pausing**: Immediate
- **Audio Resuming**: Configurable delay

### **Memory Usage**
- **VAD Buffer**: 128 bytes (64 samples × 2 bytes)
- **Status Variables**: Minimal memory footprint

## Troubleshooting

### Common Issues

1. **False Voice Detection**
   - Increase `vad_threshold`
   - Check for background noise
   - Adjust microphone positioning

2. **Missed Voice Detection**
   - Decrease `vad_threshold`
   - Check microphone sensitivity
   - Verify audio input levels

3. **Audio Cutting Out Too Quickly**
   - Increase `vad_silence_duration`
   - Check for intermittent noise
   - Adjust threshold sensitivity

4. **Audio Not Resuming**
   - Decrease `vad_silence_duration`
   - Check VAD status updates
   - Verify WebSocket connection

### Debug Information

Enable debug output by monitoring serial:
```
🎤 VOICE DETECTED - Audio paused
🔇 SILENCE DETECTED - Audio resumed
VAD Status: TALKING (RMS: 1500, Threshold: 1000)
```

## Integration with Concurrent Audio

### **Before VAD**
- Both audio streams always active
- Potential feedback issues
- User hears themselves

### **After VAD**
- Incoming audio pauses when talking
- No feedback issues
- Natural conversation flow
- Seamless audio transitions

## Configuration Recommendations

### **Quiet Environment (Office/Home)**
```json
{
    "vad_enabled": true,
    "vad_threshold": 800,
    "vad_silence_duration": 400
}
```

### **Noisy Environment (Cafe/Street)**
```json
{
    "vad_enabled": true,
    "vad_threshold": 1500,
    "vad_silence_duration": 600
}
```

### **Very Noisy Environment (Factory/Construction)**
```json
{
    "vad_enabled": true,
    "vad_threshold": 2500,
    "vad_silence_duration": 800
}
```

### **Headphones (No VAD needed)**
```json
{
    "vad_enabled": false
}
```

## API Reference

### VAD Functions
```cpp
void enableVAD(bool enable);
void setVADThreshold(int threshold);
void setVADSilenceDuration(int duration_ms);
bool detectVoiceActivity(int16_t* samples, size_t count);
void updateVADStatus();
```

### Global Variables
```cpp
extern bool vadEnabled;
extern int vadThreshold;
extern int vadSilenceDuration;
extern bool userIsTalking;
extern unsigned long lastVoiceActivity;
```

## Future Enhancements

### **Planned Features**
1. **Adaptive Threshold**: Automatic threshold adjustment
2. **Frequency Analysis**: Better voice detection using FFT
3. **Noise Cancellation**: Background noise filtering
4. **Multi-level VAD**: Different sensitivity levels

### **Performance Optimizations**
1. **SIMD Instructions**: Faster RMS calculation
2. **Optimized Buffers**: Reduced memory usage
3. **Predictive VAD**: Anticipate voice activity

## Conclusion

The VAD implementation provides:
- ✅ **Automatic audio pausing** when user talks
- ✅ **Configurable sensitivity** for different environments
- ✅ **Real-time processing** with minimal latency
- ✅ **WebSocket configuration** for remote adjustment
- ✅ **Comprehensive testing** and debugging tools

This creates a natural, feedback-free conversation experience! 🎤
