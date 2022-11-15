#ifndef _FUNCTION_CAN_H_
#define _FUNCTION_CAN_H_

#include "mbed.h"
#include "setting.h"

extern CAN can;
extern CANMessage msg;

// INIT_CID ()
#define INIT_CID_RX_CMD                   100
#define INIT_CID_RX_REF_POSITION          200
#define INIT_CID_RX_REF_VALVEPOSnPWM      300

#define INIT_CID_TX_RSP                   1100 // Response to Command
#define INIT_CID_TX_POS_VEL_TORQ          1200
#define INIT_CID_TX_VALVEPOSnPWM          1300
#define INIT_CID_TX_PRESSURE              1400
#define INIT_CID_TX_SOMETHING             1500
#define INIT_CID_TX_ALART                 1600 // Alart from Board to PC


// CID_RX_CMD - RX CMD type
#define             CRX_ASK_INFO                    0
#define             CRX_ASK_BNO                     1
#define             CRX_SET_BNO                     101
#define             CRX_ASK_OPERATING_MODE          2
#define             CRX_SET_OPERATING_MODE          102
#define             CRX_SET_ENC_ZERO                103
#define             CRX_SET_FET_ON                  104
#define             CRX_SET_POS_TORQ_TRANS          105
#define             CRX_ASK_CAN_FREQ                6
#define             CRX_SET_CAN_FREQ                106
#define             CRX_ASK_CONTROL_MODE            7
#define             CRX_SET_CONTROL_MODE            107
#define             CRX_SET_DATA_REQUEST            108
#define             CRX_ASK_JOINT_ENC_DIR           9
#define             CRX_SET_JOINT_ENC_DIR           109
#define             CRX_ASK_VALVE_DIR               10
#define             CRX_SET_VALVE_DIR               110
#define             CRX_ASK_VALVE_ENC_DIR           11
#define             CRX_SET_VALVE_ENC_DIR           111
#define             CRX_ASK_VOLTAGE_SUPPLY          12
#define             CRX_SET_VOLTAGE_SUPPLY          112
#define             CRX_ASK_VOLTAGE_VALVE           13
#define             CRX_SET_VOLTAGE_VALVE           113
#define             CRX_SET_HOMEPOS                 114  
#define             CRX_ASK_VARIABLE_SUPPLY         15      
#define             CRX_SET_VARIABLE_SUPPLY         115      
    
#define             CRX_ASK_PID_GAIN                20
#define             CRX_SET_PID_GAIN                120
#define             CRX_ASK_VALVE_DEADZONE          21
#define             CRX_SET_VALVE_DEADZONE          121
#define             CRX_ASK_VELOCITY_COMP_GAIN      22
#define             CRX_SET_VELOCITY_COMP_GAIN      122
#define             CRX_ASK_VALVE_ELECTRIC_CENTER   23 
#define             CRX_SET_VALVE_ELECTRIC_CENTER   123
#define             CRX_ASK_VALVE_FF                25
#define             CRX_SET_VALVE_FF                125
#define             CRX_ASK_BULK_MODULUS            26
#define             CRX_SET_BULK_MODULUS            126
#define             CRX_ASK_CHAMBER_VOLUME          27
#define             CRX_SET_CHAMBER_VOLUME          127
#define             CRX_ASK_PISTON_AREA             28
#define             CRX_SET_PISTON_AREA             128
#define             CRX_ASK_SUP_PRES                29
#define             CRX_SET_SUP_PRES                129
#define             CRX_ASK_ENC_LIMIT               30
#define             CRX_SET_ENC_LIMIT               130
#define             CRX_ASK_STROKE                  31
#define             CRX_SET_STROKE                  131
#define             CRX_ASK_VALVE_LIMIT             32
#define             CRX_SET_VALVE_LIMIT             132
#define             CRX_ASK_ENC_PULSE_PER_POSITION     33
#define             CRX_SET_ENC_PULSE_PER_POSITION     133
#define             CRX_ASK_TORQUE_SENSOR_PULSE_PER_TORQUE     34
#define             CRX_SET_TORQUE_SENSOR_PULSE_PER_TORQUE     134
#define             CRX_ASK_PRES_SENSOR_PULSE_PER_PRES      35
#define             CRX_SET_PRES_SENSOR_PULSE_PER_PRES      135
#define             CRX_ASK_FRICTION                36
#define             CRX_SET_FRICTION                136
#define             CRX_ASK_VALVE_GAIN_PLUS         37
#define             CRX_SET_VALVE_GAIN_PLUS         137
#define             CRX_ASK_VALVE_GAIN_MINUS        38
#define             CRX_SET_VALVE_GAIN_MINUS        138
#define             CRX_LOW_REF                     139
#define             CRX_ASK_HOMEPOS_OFFSET          40
#define             CRX_SET_HOMEPOS_OFFSET          140
#define             CRX_ASK_HOMEPOS_VALVE_OPENING   41
#define             CRX_SET_HOMEPOS_VALVE_OPENING   141
#define             CRX_ASK_DDV_VALVE_DEADZONE     42
#define             CRX_SET_DDV_VALVE_DEADZONE     142
#define             CRX_ASK_VALVE_PWM_VS_VALVE_POS  43
#define             CRX_ASK_VALVE_POS_VS_FLOWRATE   44
#define             CRX_ASK_VALVE_POS_NUM           45
#define             CRX_ASK_VALVE_MAX_MIN_POS       46
#define             CRX_SET_ERR_CLEAR               150
#define             CRX_SET_ROM                     146
#define             CRX_SET_VALVE_CENTER_OFFSET     147
#define             CRX_SET_VALVE_DZ_MINUS_OFFSET   148
#define             CRX_SET_VALVE_DZ_PLUS_OFFSET    149
#define             CRX_SET_PID_GAIN_OPP            152
#define             CRX_DELAY_TEST                  153
#define             CRX_SET_NN_CONTROL_FLAG         154
#define             CRX_SET_FREQ_TEST               155
#define             CRX_ASK_BUFFER                  156
#define             CRX_SET_STEP_TEST               157
#define             CRX_SET_CHANGE_EVERY_REFERNCE   158
#define             CRX_JUMP_STATUS                 255

// CID_TX_RSP - TX INFO type

#define             CTX_SEND_INFO                               0
#define             CTX_SEND_BNO                                1
#define             CTX_SEND_OPERATING_MODE                     2
#define             CTX_SEND_CAN_FREQ                           6
#define             CTX_SEND_CONTROL_MODE                       7
#define             CTX_SEND_JOINT_ENC_DIR                      9
#define             CTX_SEND_VALVE_DIR                          10
#define             CTX_SEND_VALVE_ENC_DIR                      11
#define             CTX_SEND_VOLTAGE_SUPPLY                     12
#define             CTX_SEND_VOLTAGE_VALVE                      13
#define             CTX_SEND_VARIABLE_SUPPLY                    15
#define             CTX_SEND_PID_GAIN                           20
#define             CTX_SEND_VALVE_DEADZONE                     21
#define             CTX_SEND_VELOCITY_COMP_GAIN                 22
#define             CTX_SEND_VALVE_ELECTRIC_CENTER              23
#define             CTX_SEND_VALVE_FF                           25
#define             CTX_SEND_BULK_MODULUS                       26
#define             CTX_SEND_CHAMBER_VOLUME                     27
#define             CTX_SEND_PISTON_AREA                        28
#define             CTX_SEND_SUP_PRES                           29
#define             CTX_SEND_ENC_LIMIT                          30
#define             CTX_SEND_STROKE                             31
#define             CTX_SEND_VALVE_LIMIT                        32
#define             CTX_SEND_ENC_PULSE_PER_POSITION             33
#define             CTX_SEND_TORQUE_SENSOR_PULSE_PER_TORQUE     34
#define             CTX_SEND_PRES_SENSOR_PULSE_PER_BAR          35
#define             CTX_SEND_FRICTION                           36
#define             CTX_SEND_VALVE_GAIN_PLUS                    37
#define             CTX_SEND_VALVE_GAIN_MINUS                   38
#define             CTX_SEND_REFENCE_MODE                       39
#define             CTX_SEND_HOMEPOS_OFFSET                     40
#define             CTX_SEND_HOMEPOS_VALVE_OPENING              41
#define             CTX_SEND_DDV_VALVE_DEADZONE                 42
#define             CTX_VALVE_PWM_VS_VALVE_POS                  43
#define             CTX_VALVE_POS_VS_FLOWRATE                   44
#define             CTX_VALVE_POS_NUM                           45
#define             CTX_VALVE_MAX_MIN_POS                       46
#define             CTX_SEND_BUFFER                             156

#define     ALART_CANERROR          0
#define     ALART_REFSTOP           1
#define     ALART_UTILFUNC_DONE     2

// Sensor & State Transmission
void CAN_TX_POSnFT(int16_t t_pos, int16_t t_vel, int16_t t_torq);
void CAN_TX_VALVEPOSnPWM(int16_t t_valve_pos, int16_t t_ref_valve_pos, int16_t t_ref_PWM);
void CAN_TX_PRESSURE(int16_t t_pres_a, int16_t t_pres_b);
void CAN_TX_SOMETHING (int16_t t_a, int16_t t_b, int16_t t_c, int16_t t_d);

// Information Transmission
void ALART_FLAG_ON(int _ALT_TYPE);
bool ALART_FLAG_CHECK(int _ALT_TYPE);
void CAN_TX_ALART(void);

void CAN_TX_INFO(void);
void CAN_TX_BNO(void);
void CAN_TX_OPERATING_MODE(void);
void CAN_TX_CAN_FREQ(void);
void CAN_TX_CONTROL_MODE(void);
void CAN_TX_JOINT_ENC_DIR(void);
void CAN_TX_VALVE_DIR(void);
void CAN_TX_VALVE_ENC_DIR(void);
void CAN_TX_VOLTAGE_SUPPLY(void);
void CAN_TX_VOLTAGE_VALVE(void);
void CAN_TX_VARIABLE_SUPPLY_ONOFF(void);

void CAN_TX_PID_GAIN(int t_type);
void CAN_TX_VALVE_DEADZONE(void);
void CAN_TX_VELOCITY_COMP_GAIN(void);
void CAN_TX_VALVE_ELECTRIC_CENTER(void);
void CAN_TX_VALVE_FF(void);
void CAN_TX_BULK_MODULUS(void);
void CAN_TX_CHAMBER_VOLUME(void);
void CAN_TX_PISTON_AREA(void);
void CAN_TX_SUP_PRES(void);
void CAN_TX_ENC_LIMIT(void);
void CAN_TX_STROKE(void);
void CAN_TX_VALVE_LIMIT(void);
void CAN_TX_ENC_PULSE_PER_POSITION(void);
void CAN_TX_TORQUE_SENSOR_PULSE_PER_TORQUE(void);
void CAN_TX_PRES_SENSOR_PULSE_PER_PRES(void);
void CAN_TX_FRICTION(void);
void CAN_TX_VALVE_GAIN_PLUS(void);
void CAN_TX_VALVE_GAIN_MINUS(void);
void CAN_TX_REFENCE_MODE(void);
void CAN_TX_HOMEPOS_OFFSET(void);
void CAN_TX_HOMPOS_VALVE_OPENING(void);
void CAN_TX_VALVE_PWM_VS_VALVE_POS(int8_t canindex);
void CAN_TX_VALVE_POS_VS_FLOWRATE(int8_t canindex);
void CAN_TX_VALVE_POS_NUM(void);
void CAN_TX_DDV_VALVE_MAX_MIN_POS(void);
void CAN_TX_BUFFER(int16_t t_cnt_buffer);

class State 
{
    public:
        float sen;
        float sen_diff;
        float ref_int; // internal reference generated by board
        float ref_ext; // external reference transmitted from PC
        float ref;
        float ref_diff;
        float err;
        float err_int;
        float err_old;
        float err_diff;
        float err_sum;
    public:
        State(){
            sen = 0.0f;
            sen_diff = 0.0f;
            ref_int = 0.0f;
            ref_ext = 0.0f;
            ref = 0.0f;
            ref_diff = 0.0f;
            err = 0.0f;
            err_int = 0.0f;
            err_old = 0.0f;
            err_diff = 0.0f;
        }
        
        void UpdateSen(float sen_new, float Freq_update, float f_cut = 1000.0f);
        void UpdateRef(float ref_new, float Freq_update, float f_cut = 500.0f);
        void Reset();
};

extern State pos;
extern State vel;
extern State force;
extern State torq;
extern State torq_dot;
extern State pres_A;
extern State pres_B;
extern State cur;
extern State valve_pos;
extern State valve_pos_raw;
extern State Vout;

// CAN Receive Functions
void ReadCMD();
void CAN_RX_HANDLER();
void CAN_ID_INIT(void);

extern float PWMs_for_ValvePosID[NUM_VALVEPOS_VS_PWM_ID];
extern float PWMs_for_ValvePosID_Sorted[NUM_VALVEPOS_VS_PWM_ID];

#endif




