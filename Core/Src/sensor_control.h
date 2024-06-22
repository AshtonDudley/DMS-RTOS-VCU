 #ifndef SENSOR_CONTROL
 #define SENSOR_CONTROL


typedef enum {
    PDP_OKAY    = 0x00U,
    PDP_ERROR   = 0x01U,
    PDP_TIMEOUT = 0x03U,
    PDP_LATCH   = 0x04U
} PDP_StatusTypeDef;

void sensor_init();

 #endif