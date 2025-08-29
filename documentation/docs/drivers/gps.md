# GPS Interface

NMEA GPS parser for position, timing, and orbital data via UART.

**Location:** `src/drivers/gps.{h,cpp}`

## Key Constants
- Buffer size: `256` bytes
- Max sentence: `82` chars
- NMEA format: Standard marine protocol

## Data Structure
```cpp
typedef struct {
    bool valid;         // Fix quality indicator
    float latitude;     // Decimal degrees (+ = North) 
    float longitude;    // Decimal degrees (+ = East)
    float altitude;     // Meters above sea level
    uint8_t satellites; // Number of satellites used
    uint32_t timestamp; // UTC time as HHMMSS
} gps_data_t;
```

## Functions
```cpp
bool gps_init(void);                    // Initialize GPS UART with interrupts
bool gps_get_data(gps_data_t *data);    // Get latest parsed GPS data
bool gps_sentence_available(void);     // Check if new NMEA sentence ready
```

## Implementation
- **Interface:** UART with interrupt-driven NMEA sentence reception
- **Parsing:** Extract lat/lon/alt/time from NMEA sentences ($GPGGA, $GPRMC)
- **Conversion:** DDMM.MMMMM format to decimal degrees
- **Output:** Position data and UTC timestamp for sun vector calculations

Used to provide GPS time (MJD) for sun vector calculation and position data for orbit determination.