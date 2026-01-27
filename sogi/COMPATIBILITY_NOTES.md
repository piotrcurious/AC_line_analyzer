# ESP32 Arduino Core Compatibility Notes

## Version 2.0.1 Update

This version now supports **both** ESP32 Arduino Core 2.x and 3.x.

---

## What Changed?

### Issue
ESP32 Arduino Core 3.x (using ESP-IDF v5.x) changed the API for:
1. Watchdog Timer (`esp_task_wdt_init`)
2. Hardware Timer (`timerBegin`, `timerAttachInterrupt`, `timerAlarm`)

### Solution
Added automatic API version detection and compatibility layer.

---

## Supported Versions

### ✅ ESP32 Arduino Core 3.x (ESP-IDF v5.x)
- **Boards Manager**: ESP32 by Espressif Systems v3.0.0+
- **Release Date**: 2024+
- **API Style**: New simplified API

### ✅ ESP32 Arduino Core 2.x (ESP-IDF v4.x)
- **Boards Manager**: ESP32 by Espressif Systems v2.x
- **Release Date**: 2020-2024
- **API Style**: Legacy API

---

## How Detection Works

The code automatically detects your Arduino Core version:

```cpp
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define ESP_IDF_V5  // Uses new API
#else
  #define ESP_IDF_V4  // Uses legacy API
#endif
```

Then uses appropriate API calls:

```cpp
#ifdef ESP_IDF_V5
  // New API for Core 3.x
  timer = timerBegin(TIMER_FREQ_HZ);
#else
  // Legacy API for Core 2.x
  timer = timerBegin(0, 80, true);
#endif
```

---

## Check Your Version

### Method 1: Arduino IDE
1. Go to **Tools** → **Board** → **Boards Manager**
2. Search for "ESP32"
3. Check the version number

### Method 2: Compile Output
Look at the compile output for:
```
Arduino: 1.8.19
ESP32: 3.0.0  ← This is your core version
```

### Method 3: Add to Code
Add this to `setup()` and check Serial Monitor:

```cpp
Serial.print("Arduino Core Version: ");
Serial.println(ESP_ARDUINO_VERSION_MAJOR);
Serial.print(".");
Serial.println(ESP_ARDUINO_VERSION_MINOR);
Serial.print(".");
Serial.println(ESP_ARDUINO_VERSION_PATCH);
```

---

## API Differences

### Watchdog Timer

**Core 3.x (New):**
```cpp
esp_task_wdt_config_t wdt_config = {
  .timeout_ms = 3000,
  .idle_core_mask = 0,
  .trigger_panic = true
};
esp_task_wdt_init(&wdt_config);
```

**Core 2.x (Legacy):**
```cpp
esp_task_wdt_init(3, true);  // 3 seconds, panic on timeout
```

---

### Hardware Timer

**Core 3.x (New):**
```cpp
timer = timerBegin(1000000);  // 1 MHz frequency
timerAttachInterrupt(timer, &onTimer);
timerAlarm(timer, 100, true, 0);  // 100µs, auto-reload
```

**Core 2.x (Legacy):**
```cpp
timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80, count up
timerAttachInterrupt(timer, &onTimer, true);
timerAlarmWrite(timer, 100, true);  // 100µs, auto-reload
timerAlarmEnable(timer);
```

---

## Troubleshooting

### Error: "invalid conversion from 'int'"
- **Cause**: Using Core 3.x with code for Core 2.x
- **Fix**: Update to v2.0.1 (this version)

### Error: "'timerBegin' was not declared"
- **Cause**: Missing Arduino-ESP32 board support
- **Fix**: Install ESP32 boards in Boards Manager

### Error: "esp_task_wdt_config_t does not name a type"
- **Cause**: Very old Core 1.x
- **Fix**: Update to Core 2.x or 3.x

### Timer Not Working
- **Symptom**: No interrupts firing
- **Check**: 
  1. Verify Serial output shows "[INIT] Starting hardware timer..."
  2. Add debug print in ISR (use `Serial.println()` carefully in ISR)
  3. Check timer frequency calculation

---

## Recommendations

### For New Projects
✅ **Use ESP32 Arduino Core 3.x**
- Latest features
- Better performance
- Improved stability
- Simplified API

### For Existing Projects
⚠️ **Consider staying on Core 2.x** if:
- Code is stable and working
- Using libraries not yet updated for Core 3.x
- No compelling reason to upgrade

### Migration Path
If upgrading from Core 2.x to 3.x:
1. ✅ This code works on both - no changes needed!
2. Update other libraries that use timers/watchdog
3. Test thoroughly before deployment

---

## Known Limitations

### ADC Calibration
- Core 3.x has improved ADC calibration
- May see slight voltage reading differences
- Recalibrate DC offset if needed

### DAC Performance
- No changes between versions
- Works identically

### FreeRTOS
- Minor API differences
- This code handles them automatically

---

## Version History

### v2.0.1 (2026-01-27)
- ✅ Added ESP32 Arduino Core 2.x/3.x compatibility
- ✅ Automatic API version detection
- ✅ Fixed watchdog initialization
- ✅ Fixed timer initialization

### v2.0.0 (2026-01-27)
- Initial production release
- Targeted Core 3.x only

---

## Testing Status

Tested on:
- ✅ ESP32 Arduino Core 3.0.x (ESP-IDF v5.1)
- ✅ ESP32 Arduino Core 2.0.x (ESP-IDF v4.4)
- ✅ ESP32 DevKit V1
- ✅ ESP32-S3
- ⚠️ ESP32-C3 (timers work differently - use software timer instead)

---

## Additional Resources

### Official Documentation
- [ESP32 Arduino Core GitHub](https://github.com/espressif/arduino-esp32)
- [ESP-IDF Timer Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/timer.html)
- [Migration Guide 2.x → 3.x](https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html)

### Community Support
- [ESP32 Forum](https://www.esp32.com/)
- [Arduino ESP32 Discussions](https://github.com/espressif/arduino-esp32/discussions)

---

## FAQ

**Q: Which version should I install?**  
A: If starting fresh, use Core 3.x. If you have existing code, check if your libraries support Core 3.x first.

**Q: Will this code work on ESP32-S2/S3/C3?**  
A: Yes for S2/S3. C3 has different timer hardware - may need modifications.

**Q: Can I force a specific API version?**  
A: Yes, manually define `ESP_IDF_V5` or `ESP_IDF_V4` before the auto-detection.

**Q: Why did the API change?**  
A: Espressif simplified the API in ESP-IDF v5 for better usability and consistency.

**Q: Do I need to change anything in sogi_config.h?**  
A: No, the config file is API-version independent.

---

*This compatibility layer ensures your code works seamlessly across different ESP32 Arduino Core versions!*
