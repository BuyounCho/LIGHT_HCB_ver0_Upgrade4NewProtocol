#include "mbed.h"
#include "FastPWM.h"

// pwm 
#define PIN_V       PB_7
#define PIN_W       PB_6
//#define PWM_ARR     0x465       // loop 80k, pwm 40k 
//#define PWM_ARR     0x8CA       // loop 40k, pwm 20k
#define PWM_ARR     0x1194      // loop 20k, pwm 10k
//#define PWM_ARR     0x2328      // loop 10k, pwm 5k
#define TMR3_COUNT  0x4650      // loop 5k
//#define TMR3_COUNT  0x2328      // loop 10k
#define TMR2_COUNT  0x2710      // loop 500hz with prescale 18

#define FREQ_500    500.0f
#define FREQ_1k     1000.0f
#define FREQ_5k     5000.0f
#define FREQ_10k    10000.0f
#define FREQ_20k    20000.0f
#define FREQ_40k    40000.0f
#define DT_500      0.002f
#define DT_1k       0.001f
#define DT_5k       0.0002f
#define DT_10k      0.0001f
#define DT_20k      0.00005f
#define DT_40k      0.000025f

//#define             TMR_FREQ_10k       10000
#define             TMR_FREQ_5k         5000

extern DigitalOut check;
extern DigitalOut check_2;
extern AnalogOut dac_1;
extern AnalogOut dac_2;

extern float dtc_v;
extern float dtc_w;

// I2C
extern I2C i2c; // SDA, SCL (for K22F)
extern const int i2c_slave_addr1; 
extern unsigned int value; // 10bit output of reading sensor AS5510

// SPI
extern SPI eeprom; //(SPI_MOSI, SPI_MISO, SPI_SCK);
extern DigitalOut eeprom_cs;
extern SPI enc;
extern DigitalOut enc_cs;

// UART 
extern Serial pc; //Serial pc(PA_9,PA_10); _ UART

// CAN
extern CAN can;
extern CANMessage msg;


/*******************************************************************************
 * COMMON Settings
 ******************************************************************************/
#define             LATEST_VERSION      19032


/*******************************************************************************
 * COMMON CONSTANTS
 ******************************************************************************/
#define             RAD_30              0.523598775598299f
#define             RAD_60              1.047197551196598f
#define             RAD_120             2.094395102393195f
#define             RAD_180             3.141592653589793f
#define             RAD_240             4.188790204786391f
#define             RAD_300             5.235987755982989f
#define             RAD_360             6.283185307179586f

#define             SYSFREQ             200000000
#define             PBCLK               100000000

#define             FALSE               0
#define             TRUE                1
#define             OUTPUT              0
#define             INPUT               1
#define             LOW                 0
#define             HIGH                1
#define             DIGITAL             0
#define             ANALOG              1

#define             PI                  3.141592653589793f
#define             D2R                 0.017453292519943f
#define             R2D                 57.295779513082323f

#define             NUM_VALVEPOS_VS_PWM_ID                 41
#define             NUM_FLOWRATE_VS_VALVEPOS_ID            51

/*******************************************************************************
 * HEADER INCLUDE
 ******************************************************************************/

/**********************************************************************
 * VARIABLE
 ******************************************************************************/

extern int16_t DebugVar_Int[16];
extern float   DebugVar_Float[16];

// Board Information 
extern uint8_t BNO;
extern uint8_t CONTROL_MODE;
extern uint8_t OPERATING_MODE;
extern uint8_t SENSING_MODE;
extern uint8_t SUPPLY_PRESSURE_UPDATE; 

extern uint8_t CONTROL_UTILITY_MODE;
extern uint8_t CURRENT_CONTROL_MODE;
extern uint8_t FLAG_VALVE_DEADZONE;
extern uint8_t REFERENCE_MODE;
extern int16_t CAN_FREQ;
extern int16_t DIR_JOINT_ENC;
extern int16_t DIR_VALVE;
extern int16_t DIR_VALVE_ENC;

extern float SUPPLY_VOLTAGE;
extern float VALVE_VOLTAGE_LIMIT;

extern float P_GAIN_VALVE_POSITION;
extern float I_GAIN_VALVE_POSITION;
extern float D_GAIN_VALVE_POSITION;
extern float P_GAIN_JOINT_POSITION;
extern float I_GAIN_JOINT_POSITION;
extern float D_GAIN_JOINT_POSITION;
extern float P_GAIN_JOINT_TORQUE;
extern float I_GAIN_JOINT_TORQUE;
extern float D_GAIN_JOINT_TORQUE;
extern float P_GAIN_JOINT_TORQUE_FF;
extern float I_GAIN_JOINT_TORQUE_FF;
extern float D_GAIN_JOINT_TORQUE_FF;

extern float K_SPRING;
extern float D_DAMPER;


extern int16_t VALVE_DEADZONE_PLUS;
extern int16_t VALVE_DEADZONE_MINUS;

extern int16_t VELOCITY_COMP_GAIN;
//extern int16_t COMPLIANCE_GAIN;

extern int16_t VALVE_CENTER;

extern int16_t VALVE_FF;

extern int16_t BULK_MODULUS;

extern int16_t CHAMBER_VOLUME_A;
extern int16_t CHAMBER_VOLUME_B;

extern int16_t PISTON_AREA_A;
extern int16_t PISTON_AREA_B;
extern float PISTON_AREA_alpha;
extern float alpha3;

extern float PRES_SUPPLY_NOM;
extern float PRES_SUPPLY;

extern int16_t ENC_LIMIT_PLUS;
extern int16_t ENC_LIMIT_MINUS;

extern int16_t STROKE;

extern float Amm;
extern float beta;
extern float Ps;
extern float Pt;
extern float gamma_hat;
extern float a_hat;
extern float V_adapt;
extern float x_4_des_old;

//extern int16_t VALVE_LIMIT_PLUS;
//extern int16_t VALVE_LIMIT_MINUS;

extern float ENC_PULSE_PER_POSITION;
extern float TORQUE_SENSOR_PULSE_PER_TORQUE;

extern float PRES_SENSOR_A_PULSE_PER_BAR;
extern float PRES_SENSOR_B_PULSE_PER_BAR;

extern int16_t HOMEPOS_OFFSET;
extern int HOMEPOS_VALVE_OPENING;

extern float FRICTION;
extern float REF_PERIOD;
extern float REF_MAG;
extern int REF_NUM;


extern float DAC_REF;
extern float DAC_RESOL;

extern int16_t REF_PWM;
extern int16_t REF_VALVE_POSITION;
extern int16_t REF_CURRENT;

extern int REF_MOVE_TIME_5k;
extern int INIT_REF_PWM;
extern int INIT_REF_VALVE_POS;
extern int INIT_REF_VEL;
extern int INIT_REF_TORQUE;
extern int INIT_REF_PRES_DIFF;
extern int INIT_REF_CURRENT;

extern unsigned int    TMR2_COUNT_LED1;
extern unsigned int    TMR2_COUNT_LED2;
extern unsigned int    TMR2_COUNT_CAN_TX;
extern unsigned int    TMR3_COUNT_TEST;

extern int flag_alart[24];
extern int flag_alart_old[24];

extern int flag_ref_enable;

extern int flag_data_request[4];

extern int MODE_POS_FT_TRANS;
extern int NN_Control_Flag;

extern int cnt_buffer;

extern float CUR_CURRENT_mA;
extern float CUR_TORQUE_NM;
extern float CUR_TORQUE_NM_PRESS;

extern float FORCE_VREF;
extern float FORCE_B_VREF;
extern float PRES_A_VREF;
extern float PRES_B_VREF;

extern float VALVE_PWM_RAW_FB;
extern float VALVE_PWM_RAW_FF;
extern float VALVE_PWM_RAW;
extern int VALVE_PWM_VALVE_DZ;

extern float VALVE_GAIN_LPM_PER_V[10];

extern int VALVE_MAX_POS;
extern int VALVE_MIN_POS;
extern int VALVE_ELECTRIC_CENTER;
extern int VALVE_POS_NUM;
extern float VALVE_CENTER_OFFSET;
extern float VALVE_DZ_MINUS_OFFSET;
extern float VALVE_DZ_PLUS_OFFSET;

extern int TMR3_COUNT_FINDHOME;
extern int TMR3_COUNT_FLOWRATE;
extern int TMR3_COUNT_DEADZONE;
extern int TMR3_COUNT_PRES_NULL;
extern int TMR3_COUNT_TORQUE_NULL;
extern int TMR3_COUNT_PRES_CALIB;
extern int TMR3_COUNT_REFERENCE;
extern int TMR3_COUNT_JOINT;
extern int TMR3_COUNT_ROTARY_FRIC_TUNE;

extern float TUNING_TIME;

extern float REFERENCE_FREQ;
extern float REFERENCE_MAG;

extern bool FLAG_FIND_HOME;

extern int MODE_JUMP_STATUS;

extern float CUR_PRES_DIFF_BAR;
extern float CUR_PRES_A_sum;
extern float CUR_PRES_B_sum;
extern float CUR_PRES_A_mean;
extern float CUR_PRES_B_mean;
extern float PRES_A_NULL_pulse;
extern float PRES_B_NULL_pulse;
extern float FORCE_NULL_pulse;

extern float Ref_Valve_Pos_Old;

extern int FINDHOME_STAGE;

extern int VALVE_FR_timer;
extern int DDV_POS_AVG;
extern int data_num;

extern float PWMs_for_ValvePosID[NUM_VALVEPOS_VS_PWM_ID];
extern float PWMs_for_ValvePosID_Sorted[NUM_VALVEPOS_VS_PWM_ID];
extern float VALVE_POS_SUM;
extern float VALVE_POS_AVG_OLD;
extern float VALVE_POS_AVG[NUM_VALVEPOS_VS_PWM_ID];
extern float VALVE_POS_VS_PWM[NUM_VALVEPOS_VS_PWM_ID];
extern float VALVE_POS_VS_PWM_Sorted[NUM_VALVEPOS_VS_PWM_ID];

extern float ValvePosOff_for_FlowrateID[NUM_FLOWRATE_VS_VALVEPOS_ID];
extern float ValvePosOff_for_FlowrateID_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID];
extern float FlowRate_VS_ValvePosOff[NUM_FLOWRATE_VS_VALVEPOS_ID];
extern float FlowRate_VS_ValvePosOff_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID];

extern int first_check;
extern int DZ_case;
extern float START_POS;
extern float FINAL_POS;
extern int DZ_DIRECTION;
extern float DZ_OPENSIZE;
extern float INIT_DZ_OPENSIZE;
extern int FIRST_DZ;
extern int SECOND_DZ;
extern int DZ_NUM;

extern float valve_pos_err, valve_pos_err_old, valve_pos_err_diff, valve_pos_err_sum;
extern float joint_pos_err, joint_pos_err_old, joint_pos_err_diff, joint_pos_err_diff_fil, joint_pos_err_sum;
extern float joint_torq_err, joint_torq_err_old, joint_torq_err_diff, joint_torq_err_sum;
extern float VALVE_PWM_RAW_POS, VALVE_PWM_RAW_TORQ;

extern int pos_plus_end;
extern int pos_minus_end;
extern int pos_mid;

extern bool need_enc_init;

extern float CUR_VELOCITY_sum;
extern float temp_vel_sum;

extern int cnt_finddz;
extern int cnt_vel_finddz;
extern int flag_finddz;
extern int FINDDZ_VELOCITY;
extern int FINDDZ_VELOCITY_OLD;
extern int FINDDZ_POSITION;
extern int FINDDZ_POSITION_OLD;

extern double temp_VALVE_DEADZONE_PLUS;
extern double temp_VALVE_DEADZONE_MINUS;
extern float temp_pos_ref;
extern float temp_pos_ref_offset;

// valve gain
extern int check_vel_pos_init;
extern int check_vel_pos_fin;
extern int check_vel_pos_interv;
extern int valve_gain_repeat_cnt;
extern float VALVE_VOLTAGE;

extern float freq_fric_tune;

extern uint32_t TMR3_COUNT_CAN_TX;

// Current Control VariablesZ
extern double I_REF;
extern double I_REF_fil;
extern double I_REF_fil_DZ;
extern double I_ERR;
extern double I_ERR_INT;
extern double I_REF_fil_old;
extern double I_REF_fil_diff;

// system id
extern int cnt_sysid;
extern double freq_sysid_Iref;

extern int cnt_freq_test;
extern int cnt_step_test;
extern int buffer_data_size;
extern int cnt_send_buffer;
extern float freq_test_valve_ref;
extern float ref_array[10000];
extern int pos_array[10000];
extern int flag_every_reference;

extern int TMR3_COUNT_IREF;
extern float CUR_CURRENT;
extern float u_CUR[3];

extern float alpha_trans;

extern float V_out;
extern float V_rem;
extern float V_MAX;

extern float PWM_out;

extern double K_v;
extern double C_d;
extern double mV_PER_mA;
extern double mV_PER_pulse;
extern double mA_PER_pulse;

extern int timer_while;
extern int while_index;
extern int RL_timer;

extern float K_LPF;
extern float D_LPF;

extern float V_EXI;









