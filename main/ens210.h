#ifndef ENS210_H
#define ENS210_H

#define ENS210_ADDR 0x43

// Registers
#define ENS210_PART_ID    0x00
#define ENS210_DIE_REV    0x02
#define ENS210_UID        0x04
#define ENS210_SYS_CTRL   0x10
#define ENS210_SYS_STAT   0x11
#define ENS210_SENS_RUN   0x21
#define ENS210_SENS_START 0x22
#define ENS210_SENS_STOP  0x23
#define ENS210_SENS_STAT  0x24
#define ENS210_T_VAL      0x30
#define ENS210_H_VAL      0x33

#define ENS210_LOW_POWER_MODE_DISABLED 0x00
#define ENS210_LOW_POWER_MODE_ENABLED  0x01
#define ENS210_RESET                    0x80

#define DEVICE_ID_ENS210              0x0210

#define ENS210_STATUS_DATA_VALID 1

// 7654 3211
// Polynomial 0b 1000 1001 ~ x^7+x^3+x^0
// 0x 8 9
#define CRC7WIDTH 7     // 7 bits CRC has polynomial of 7th order (has 8 terms)
#define CRC7POLY  0x89  // The 8 coefficients of the polynomial
#define CRC7IVEC  0x7F  // Initial vector has all 7 bits high
// Payload data
#define DATA7WIDTH 17
#define DATA7MASK  ((1UL << DATA7WIDTH) - 1)  // 0b 0 1111 1111 1111 1111
#define DATA7MSB   (1UL << (DATA7WIDTH - 1))  // 0b 1 0000 0000 0000 0000

typedef enum { ENS210_RUN_MODE_SINGLE_SHOT = 0x00, ENS210_RUN_MODE_CONTINUOUS } ens210_run_mode_t;
#endif // ENS210_H