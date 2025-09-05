# System Main & Initialization

Main entry point with hardware setup and infinite scheduler loop.

**Location:** `src/main.cpp` + `src/init.{h,cpp}`

## Global State
```cpp
slate_t slate;  // Global system state structure
```

## Main Flow
```cpp
stdio_init_all();       // Initialize I/O interfaces
sleep_ms(5000);         // Hardware settling delay
init(&slate);           // Hardware initialization  
sched_init(&slate);     // State machine initialization
while (1) {             // Main loop
    sched_dispatch(&slate);
}
```

## Implementation
- **Hardware check:** `static_assert(PICO_RP2350A == 0, ...)` ensures correct build target
- **Memory report:** Logs slate structure size for debugging
- **Error handling:** `ERROR()` macro if main loop exits (should never happen)

Foundation for all ADCS operations with continuous scheduler execution.