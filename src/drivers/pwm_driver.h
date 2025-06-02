#define MAX_CURRENT 400 // Maximum current in mA
#define V_REF 
#define R_REF

#define pwm_initialized

// Prototypes for the functions we will implement
uint8_t init_pwm(i2c_t i2c0)
uint8_t do_pwm(int8_t xdn, int8_t ydn, int8_t zdn, max_current); //int is signed, uint is unsigned

