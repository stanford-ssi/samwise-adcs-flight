# Watchdog Timer

Hardware watchdog timer to prevent system hangs and enable recovery from software faults.

**Location:** `src/drivers/watchdog.{h,cpp}`

## Functions
```cpp
bool watchdog_init(slate_t *slate);   // Initialize watchdog system
void watchdog_feed(slate_t *slate);   // Feed watchdog to prevent reset
```

## Implementation
- **Purpose:** Automatically reset system if software hangs
- **Feeding:** Must be called periodically to prevent timeout
- **Integration:** Called by watchdog task in scheduler

Used by system to ensure continuous operation and automatic recovery from faults.