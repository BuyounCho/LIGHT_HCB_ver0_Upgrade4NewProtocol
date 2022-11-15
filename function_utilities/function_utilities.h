#ifndef _FUNCTION_UTILITIES_H
#define _FUNCTION_UTILITIES_H

//address (write & read)
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

// sector (erase sector) binary_SNB<<3
#define FLASH_Latency_0                ((uint8_t)0x0000)  /*!< FLASH Zero Latency cycle      */
#define FLASH_Latency_1                ((uint8_t)0x0001)  /*!< FLASH One Latency cycle       */
#define FLASH_Latency_2                ((uint8_t)0x0002)  /*!< FLASH Two Latency cycles      */
#define FLASH_Latency_3                ((uint8_t)0x0003)  /*!< FLASH Three Latency cycles    */
#define FLASH_Latency_4                ((uint8_t)0x0004)  /*!< FLASH Four Latency cycles     */
#define FLASH_Latency_5                ((uint8_t)0x0005)  /*!< FLASH Five Latency cycles     */
#define FLASH_Latency_6                ((uint8_t)0x0006)  /*!< FLASH Six Latency cycles      */

float   dabs(float tx);
float   change_int_to_efloat(int input);
void     make_delay(void);
void    ArrayMasking_1st(float *in, float *out, const int n, float mask[3]);
void    ArrayMasking_2nd(float *in, float *out, const int n, float mask[5]);

void     ROM_CALL_DATA(void);

void     ENC_UPDATE(void);
void     ENC_SET_ZERO(void);
void     ENC_SET(long value);
void     VALVE_POSITION_INIT(void);
void     VALVE_PWM(int pwm, float vol_max, float vol_in);

#define     VALVETYPE_MOOG      0
#define     VALVETYPE_KNR       1

#define     ACTUATORTYPE_ROT    0
#define     ACTUATORTYPE_LIN    1

int Classify_ValveType(void);
int Classify_ActuatorType(void);

#endif
