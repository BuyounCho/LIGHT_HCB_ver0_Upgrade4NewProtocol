#include "setting.h"
#include "SPI_EEP_ENC.h"
#include "function_utilities.h"
#include "function_CAN.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"

/*******************************************************************************
 * VARIABLE
 ******************************************************************************/

// Debugging Variables
int16_t DebugVar_Int[16];
float   DebugVar_Float[16];

// Board Information
uint8_t BNO = 0;
uint8_t CONTROL_MODE = 0;
uint8_t OPERATING_MODE = 0; // (00 : Moog & Rot, 01 : Moog & Lin, 10 : KNR & Rot, 11 : KNR & Lin, 101 : SW & Lin)
uint8_t SENSING_MODE = 0; // (0 : torque, 1: pressure)
uint8_t SUPPLY_PRESSURE_UPDATE = 0; // (0 : Update Off (constant Ps) , 1 : Update On (variable Ps))

uint8_t CONTROL_UTILITY_MODE = 0;
uint8_t CURRENT_CONTROL_MODE = 0; // (0 : pwm, 1 : current control)
uint8_t FLAG_VALVE_DEADZONE = 0;
uint8_t REFERENCE_MODE = 1;
int16_t CAN_FREQ = 500;
int16_t DIR_JOINT_ENC = 1;
int16_t DIR_VALVE = 0;
int16_t DIR_VALVE_ENC = 0;

float SUPPLY_VOLTAGE = 12.0f;
float VALVE_VOLTAGE_LIMIT = 12.0f;  //v

float P_GAIN_VALVE_POSITION = 0.0f;
float I_GAIN_VALVE_POSITION= 0.0f;
float D_GAIN_VALVE_POSITION= 0.0f;
float P_GAIN_JOINT_POSITION = 0.0f;
float I_GAIN_JOINT_POSITION = 0.0f;
float D_GAIN_JOINT_POSITION = 0.0f;
float P_GAIN_JOINT_TORQUE = 0.0f;
float I_GAIN_JOINT_TORQUE = 0.0f;
float D_GAIN_JOINT_TORQUE = 0.0f;
float P_GAIN_JOINT_TORQUE_FF = 0.0f;
float I_GAIN_JOINT_TORQUE_FF = 0.0f;
float D_GAIN_JOINT_TORQUE_FF = 0.0f;

float K_SPRING = 0.0f;
float D_DAMPER = 0.0f;

int16_t VALVE_DEADZONE_PLUS;
int16_t VALVE_DEADZONE_MINUS;

int16_t VELOCITY_COMP_GAIN;

int16_t VALVE_CENTER;

int16_t VALVE_FF;

int16_t BULK_MODULUS;

int16_t CHAMBER_VOLUME_A;
int16_t CHAMBER_VOLUME_B;

int16_t PISTON_AREA_A;
int16_t PISTON_AREA_B;
float PISTON_AREA_alpha;
float alpha3 = 1.0f;

float PRES_SUPPLY_NOM = 70.0f;
float PRES_SUPPLY = 70.0f;

int16_t ENC_LIMIT_PLUS;
int16_t ENC_LIMIT_MINUS;

int16_t STROKE;

float Amm = 236.4f;
float beta = 1300000000.0f;
float Ps = 10000000.0f; //100bar = 100*10^5 Pa
float Pt = 0.0f;    // 0bar = 0Pa
//float Kv = 0.00000002635f;          // Q = Kv*xv*sqrt(Ps-Pa)    => 100bar full opening 5LPM    (full opening : xv = 1)  [unit] m^3.5/kg^0.5
float gamma_hat = 1075.0f; // Kv*beta*A/(sqrt(2)*V)   0.00000002635f * 1300000000.0f *  / (sqrt(2.0f)*(1256.6f + 236.4f * 39.75f) * 0.000000001f / 2)     [unit] m^3.5/kg^0.5
float a_hat = -13707631.7f;
float V_adapt = 0.0000053f;           // (1256.6f + 236.4f * 39.75f) * 0.000000001f / 2
float x_4_des_old = 0.0f;

float ENC_PULSE_PER_POSITION = 1.0f;
float TORQUE_SENSOR_PULSE_PER_TORQUE = 1.0f;
float PRES_SENSOR_A_PULSE_PER_BAR = 4096.0f / 200.0f;
float PRES_SENSOR_B_PULSE_PER_BAR = 4096.0f / 200.0f;

int16_t HOMEPOS_OFFSET;
int HOMEPOS_VALVE_OPENING;

float FRICTION;
float REF_PERIOD;
float REF_MAG;
int REF_NUM;


float DAC_REF;
float DAC_RESOL;

int16_t REF_PWM;
int16_t REF_VALVE_POSITION;
int16_t REF_CURRENT;

int REF_MOVE_TIME_5k;
int INIT_REF_PWM;
int INIT_REF_VALVE_POS;
int INIT_REF_VEL;
int INIT_REF_TORQUE;
int INIT_REF_PRES_DIFF;
int INIT_REF_CURRENT;

unsigned int    TMR2_COUNT_LED1;
unsigned int    TMR2_COUNT_LED2;
unsigned int    TMR2_COUNT_CAN_TX = 0;
unsigned int    TMR3_COUNT_TEST = 0;

int flag_alart[24];
int flag_alart_old[24];

int flag_ref_enable;

int flag_data_request[4];

int MODE_POS_FT_TRANS = 0;
int NN_Control_Flag = 0;

int cnt_buffer = 0;

float CUR_CURRENT_mA = 0.0f;
float CUR_TORQUE_NM = 0.0f;
float CUR_TORQUE_NM_PRESS = 0.0f;

float FORCE_VREF = 0.0f;
float FORCE_B_VREF = 0.0f;
float PRES_A_VREF = 0.0f;
float PRES_B_VREF = 0.0f;

float VALVE_PWM_RAW_FB = 0.0f;
float VALVE_PWM_RAW_FF = 0.0f;
float VALVE_PWM_RAW = 0.0f;
int VALVE_PWM_VALVE_DZ = 0;

float VALVE_GAIN_LPM_PER_V[10];

int VALVE_MAX_POS;
int VALVE_MIN_POS;
int VALVE_ELECTRIC_CENTER;
int VALVE_POS_NUM;
float VALVE_CENTER_OFFSET;
float VALVE_DZ_MINUS_OFFSET;
float VALVE_DZ_PLUS_OFFSET;

int TMR3_COUNT_FINDHOME = 0;
int TMR3_COUNT_FLOWRATE = 0;
int TMR3_COUNT_DEADZONE = 0;
int TMR3_COUNT_PRES_NULL = 0;
int TMR3_COUNT_TORQUE_NULL = 0;
int TMR3_COUNT_PRES_CALIB = 0;
int TMR3_COUNT_REFERENCE = 0;
int TMR3_COUNT_JOINT = 0;
int TMR3_COUNT_ROTARY_FRIC_TUNE = 0;

float TUNING_TIME = 600.0f;  // sec

float REFERENCE_FREQ = 1.0f;
float REFERENCE_MAG = 0.0f;

bool FLAG_FIND_HOME;

int MODE_JUMP_STATUS;
enum _JUMP_STATUS {
    JUMP_NO_ACT = 0,                                //0
    JUMP_START,                                //1
    JUMP_TAKEOFF,                                  //2
    JUMP_FLYING,                                 //3
    JUMP_LANDING,                                  //4
};


float PRES_A_NULL_pulse = 300.0f;
float PRES_B_NULL_pulse = 300.0f;
float FORCE_NULL_pulse = 3900.0f;

float Ref_Valve_Pos_Old = 0.0f;

int VALVE_FR_timer = 0;
int DDV_POS_AVG = 0;
int data_num = 0;
int DZ_index = 1;

float PWMs_for_ValvePosID[NUM_VALVEPOS_VS_PWM_ID] = {0.0f};
float PWMs_for_ValvePosID_Sorted[NUM_VALVEPOS_VS_PWM_ID] = {0.0f};
float VALVE_POS_SUM = 0;
float VALVE_POS_AVG[NUM_VALVEPOS_VS_PWM_ID] = {0};
float VALVE_POS_AVG_OLD = 0;
float VALVE_POS_VS_PWM[NUM_VALVEPOS_VS_PWM_ID];
float VALVE_POS_VS_PWM_Sorted[NUM_VALVEPOS_VS_PWM_ID];

float ValvePosOff_for_FlowrateID[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f};
float ValvePosOff_for_FlowrateID_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f};
float FlowRate_VS_ValvePosOff[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f};
float FlowRate_VS_ValvePosOff_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f};

int first_check = 0;
int DZ_case = 1;
float START_POS = 0.0f;
float FINAL_POS = 0.0f;
int DZ_DIRECTION = 0;
float DZ_OPENSIZE = 128.0f;
float INIT_DZ_OPENSIZE = 128.0f;
int FIRST_DZ = 0;
int SECOND_DZ = 0;
int DZ_NUM = 0;

float valve_pos_err = 0.0f, valve_pos_err_old = 0.0f, valve_pos_err_diff = 0.0f, valve_pos_err_sum = 0.0f;
float joint_pos_err = 0.0f, joint_pos_err_old = 0.0f, joint_pos_err_diff = 0.0f, joint_pos_err_diff_fil = 0.0f, joint_pos_err_sum = 0.0f;
float joint_torq_err = 0.0f, joint_torq_err_old = 0.0f, joint_torq_err_diff = 0.0f, joint_torq_err_sum = 0.0f;
float VALVE_PWM_RAW_POS = 0.0f, VALVE_PWM_RAW_TORQ = 0.0f;

int pos_plus_end = 0;
int pos_minus_end = 0;
int pos_mid = 0; 

bool need_enc_init = false;

float CUR_VELOCITY_sum = 0.0f;
float temp_vel_sum = 0.0f;

int cnt_finddz = 0;
int cnt_vel_finddz = 0;
int flag_finddz = 0;
int FINDDZ_VELOCITY = 0;
int FINDDZ_VELOCITY_OLD = 0;
int FINDDZ_POSITION = 0;
int FINDDZ_POSITION_OLD = 0;

double temp_VALVE_DEADZONE_PLUS = 0.0f;
double temp_VALVE_DEADZONE_MINUS = 0.0f;
float temp_pos_ref = 0.0f;
float temp_pos_ref_offset = 0.0f;


// valve gain
int check_vel_pos_init = 0;
int check_vel_pos_fin = 0;
int check_vel_pos_interv = 0;
int valve_gain_repeat_cnt = 0;
float VALVE_VOLTAGE = 0.0f;

float freq_fric_tune = 1.0f;

uint32_t TMR3_COUNT_CAN_TX = 0;

// Current Control Variables
double I_REF = 0.0f;
double I_REF_fil = 0.0f;
double I_REF_fil_DZ = 0.0f;
double I_ERR = 0.0f;
double I_ERR_INT = 0.0f;
double I_REF_fil_old = 0.0f;
double I_REF_fil_diff = 0.0f;

// system id
int cnt_sysid = 0;
double freq_sysid_Iref = 0.0f;

int cnt_freq_test = 0;
int cnt_step_test = 0;
int buffer_data_size = 0;
int cnt_send_buffer = 0;
float freq_test_valve_ref = 1.0f;
float ref_array[10000];
int pos_array[10000];
int flag_every_reference = 0;

int TMR3_COUNT_IREF = 0;
float CUR_CURRENT = 0.0f;
float u_CUR[3] = {0.0f,0.0f,0.0f};

float alpha_trans = 0.0f;

float V_out=0.0f;
float PWM_out=0.0f;

double K_v = 1.0f; // valve flowrate gain 1
double C_d = 0.16f; // valve flowrate gain 2

double mV_PER_mA = 600.0f; // current >> voltage
double mV_PER_pulse = 0.6f; // pulse >> voltage
double mA_PER_pulse = 0.001f; // pulse >> current

int timer_while = 0;
int while_index = 0;
int RL_timer = 0;

float K_LPF = 0.0f;
float D_LPF = 0.0f;

float V_EXI = 0.0f;

/*******************************************************************************
 * General math functions
 ******************************************************************************/


float dabs(float tx)
{
    if (tx >= 0.0f)
        return tx;
    else
        return -tx;
}

float tanh_inv(float y) {
if (y >= 1.0f - 0.000001f)
    y = 1.0f - 0.000001f;
if (y <= -1.0f + 0.000001f)
    y = -1.0f + 0.000001f;
return log(sqrt((1.0f + y) / (1.0f - y)));
}

float change_int_to_efloat(int input)
{
    int i = 0;

    float output = 0;
    int vn = (int) ((float) input / 10.0f);
    int en = input % 10;

    float temp = 1.;
    for (i = 0; i < en; i++)
        temp *= 0.1f;

    output = (float) vn*temp;
    return output;
}

void ArrayMasking_1st(float *in, float *out, const int n, float mask[3]) {
    if(n<3) {
        return;
    } else {
        out[0] = in[0];
        for(int i=1;i<n-1;i++) {
            out[i] = mask[0]*in[i-1] + mask[1]*in[i] + mask[2]*in[i+1];
        }
        out[n-1] = in[n-1];
    }
}

void ArrayMasking_2nd(float *in, float *out, const int n, float mask[5]) {
    if(n<5) {
        return;
    } else {
        out[0] = in[0];
        out[1] = in[1];
        for(int i=2;i<n-2;i++) {
            out[i] = mask[0]*in[i-2] + mask[1]*in[i-1] + mask[2]*in[i] + mask[3]*in[i+1] + mask[4]*in[i+2];
        }
        out[n-2] = in[n-2];
        out[n-1] = in[n-1];
    }
}

void make_delay(void)
{
    int i = 0;

    for (i = 0; i < 1000000; i++) {
        ;
    }
}


/*******************************************************************************
 * ROM functions
 ******************************************************************************/

void ROM_CALL_DATA(void)
{
    BNO = spi_eeprom_read(RID_BNO);
    OPERATING_MODE = spi_eeprom_read(RID_OPERATING_MODE);

    SENSING_MODE = spi_eeprom_read(RID_SENSING_MODE);
    CURRENT_CONTROL_MODE = spi_eeprom_read(RID_CURRENT_CONTROL_MODE);
    FLAG_VALVE_DEADZONE = spi_eeprom_read(RID_FLAG_VALVE_DEADZONE);
    CAN_FREQ = spi_eeprom_read(RID_CAN_FREQ);
    DIR_JOINT_ENC = spi_eeprom_read(RID_JOINT_ENC_DIR);
    DIR_VALVE = spi_eeprom_read(RID_VALVE_DIR);
    DIR_VALVE_ENC = spi_eeprom_read(RID_VALVE_ENC_DIR);
    SUPPLY_VOLTAGE = (float) (spi_eeprom_read(RID_VOLATGE_SUPPLY)) *0.1f;
    VALVE_VOLTAGE_LIMIT = (float) (spi_eeprom_read(RID_VOLTAGE_VALVE)) * 0.1f;
    P_GAIN_VALVE_POSITION = spi_eeprom_read(RID_P_GAIN_VALVE_POSITION);
    I_GAIN_VALVE_POSITION = spi_eeprom_read(RID_I_GAIN_VALVE_POSITION);
    D_GAIN_VALVE_POSITION = spi_eeprom_read(RID_D_GAIN_VALVE_POSITION);
    P_GAIN_JOINT_POSITION = spi_eeprom_read(RID_P_GAIN_JOINT_POSITION);
    I_GAIN_JOINT_POSITION = spi_eeprom_read(RID_I_GAIN_JOINT_POSITION);
    D_GAIN_JOINT_POSITION = spi_eeprom_read(RID_D_GAIN_JOINT_POSITION);
    P_GAIN_JOINT_TORQUE = spi_eeprom_read(RID_P_GAIN_JOINT_TORQUE);
    I_GAIN_JOINT_TORQUE = spi_eeprom_read( RID_I_GAIN_JOINT_TORQUE);
    D_GAIN_JOINT_TORQUE = spi_eeprom_read(RID_D_GAIN_JOINT_TORQUE);
    VALVE_DEADZONE_PLUS = (spi_eeprom_read(RID_VALVE_DEADZONE_PLUS));
    VALVE_DEADZONE_MINUS = (spi_eeprom_read(RID_VALVE_DEADZONE_MINUS));
    VELOCITY_COMP_GAIN = spi_eeprom_read(RID_VELOCITY_COMP_GAIN);
//    COMPLIANCE_GAIN = spi_eeprom_read(RID_COMPLIANCE_GAIN);
    VALVE_CENTER = spi_eeprom_read(RID_VALVE_CENTER);
    VALVE_FF = spi_eeprom_read(RID_VALVE_FF);
    BULK_MODULUS = spi_eeprom_read(RID_BULK_MODULUS);
    CHAMBER_VOLUME_A = spi_eeprom_read(RID_CHAMBER_VOLUME_A);
    CHAMBER_VOLUME_B = spi_eeprom_read(RID_CHAMBER_VOLUME_B);
    PISTON_AREA_A = spi_eeprom_read(RID_PISTON_AREA_A);
    PISTON_AREA_B = spi_eeprom_read(RID_PISTON_AREA_B);
    PISTON_AREA_alpha = (float)PISTON_AREA_A/(float)PISTON_AREA_B;
    alpha3 = PISTON_AREA_alpha * PISTON_AREA_alpha*PISTON_AREA_alpha;
    PRES_SUPPLY_NOM = spi_eeprom_read(RID_PRES_SUPPLY);
    PRES_SUPPLY = PRES_SUPPLY_NOM;
    ENC_LIMIT_MINUS = spi_eeprom_read(RID_ENC_LIMIT_MINUS);
    ENC_LIMIT_PLUS = spi_eeprom_read(RID_ENC_LIMIT_PLUS);
    STROKE = spi_eeprom_read(RID_STROKE);
    ENC_PULSE_PER_POSITION = (float) (spi_eeprom_read(RID_ENC_PULSE_PER_POSITION));
    TORQUE_SENSOR_PULSE_PER_TORQUE = (float) (spi_eeprom_read(RID_TORQUE_SENSOR_PULSE_PER_TORQUE)) * 0.001f;
    PRES_SENSOR_A_PULSE_PER_BAR = (float) (spi_eeprom_read(RID_PRES_SENSOR_A_PULSE_PER_BAR)) * 0.01f;
    PRES_SENSOR_B_PULSE_PER_BAR = (float) (spi_eeprom_read(RID_PRES_SENSOR_B_PULSE_PER_BAR)) * 0.01f;
    FRICTION = (float) (spi_eeprom_read(RID_FRICTION)) * 0.1f;
    HOMEPOS_OFFSET = spi_eeprom_read(RID_HOMEPOS_OFFSET);
    HOMEPOS_VALVE_OPENING = spi_eeprom_read(RID_HOMEPOS_VALVE_OPENING);
    FORCE_VREF = (float) (spi_eeprom_read(RID_FORCE_SENSOR_VREF)) *0.001f;
    FORCE_B_VREF = (float) (spi_eeprom_read(RID_FORCE_SENSOR_VREF_B)) *0.001f;
    PRES_A_VREF = (float) spi_eeprom_read(RID_PRES_A_SENSOR_VREF) * 0.001f;
    PRES_B_VREF = (float) spi_eeprom_read(RID_PRES_B_SENSOR_VREF) * 0.001f;
    // VALVE_GAIN_LPM_PER_V[0] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_1)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[2] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_2)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[4] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_3)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[6] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_4)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[8] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_5)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[1] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_1)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[3] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_2)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[5] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_3)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[7] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_4)) * 0.01f;
    // VALVE_GAIN_LPM_PER_V[9] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_5)) * 0.01f;
    for(int i=0; i<NUM_VALVEPOS_VS_PWM_ID; i++) {
        VALVE_POS_VS_PWM_Sorted[i] = (float)(spi_eeprom_read(RID_VALVE_POS_VS_PWM_0 + i));
    }
    for(int i=0; i<NUM_FLOWRATE_VS_VALVEPOS_ID; i++) {
        FlowRate_VS_ValvePosOff_Sorted[i] = (float) (spi_eeprom_read(RID_VALVE_POS_VS_FLOWRATE_0 + i))/1000.0f;
    }

//    K_SPRING = spi_eeprom_read(RID_K_SPRING);
//    D_DAMPER = spi_eeprom_read(RID_D_DAMPER);

}

/*******************************************************************************
 * ENCODER functions
 ******************************************************************************/
long ENC_pulse = 0, ENC_pulse_old = 0, ENC_pulse_diff = 0;
long ENC_pulse_offset = 0;

void ENC_UPDATE(void)
{
    ENC_pulse = spi_enc_read(); // Unit : pulse
    ENC_pulse_diff = ENC_pulse - ENC_pulse_old;

    pos.UpdateSen((float)((long)DIR_JOINT_ENC * ENC_pulse + ENC_pulse_offset)/ENC_PULSE_PER_POSITION, FREQ_10k, 200.0f); // Unit : deg or mm
    vel.UpdateSen((float)((long)DIR_JOINT_ENC * ENC_pulse_diff * (long)FREQ_10k)/ENC_PULSE_PER_POSITION, FREQ_10k, 200.0f); // Unit : deg/s or mm/s

    ENC_pulse_old = ENC_pulse;
}

void ENC_SET_ZERO(void)
{
    spi_enc_set_clear();
    pos.Reset();
    ENC_pulse_offset = 0;
    ENC_pulse = ENC_pulse_old = ENC_pulse_diff = 0;
}

void ENC_SET(long value_e)
{
    spi_enc_set_clear();
    ENC_pulse_offset = value_e;
    pos.Reset();
    pos.sen = value_e/ENC_PULSE_PER_POSITION; // deg or mm
    ENC_pulse = ENC_pulse_old = value_e;
    ENC_pulse_diff = 0;
}

/*******************************************************************************
 * Operating Mode Classification
 ******************************************************************************/

int Classify_ValveType(void) {
    if(((OPERATING_MODE & 0b0110)>>1) == VALVETYPE_MOOG) { // Moog Two-stage Valve
        return 0;
    } else if(((OPERATING_MODE & 0b0110)>>1) == VALVETYPE_KNR) { // KNR Two-stage Valve
        return 1;
    } else {
        return -1;
    }
}

int Classify_ActuatorType(void) {
    if(((OPERATING_MODE & 0b0001)) == ACTUATORTYPE_ROT) { // Rotary Actuator
        return 0;
    } else if(((OPERATING_MODE & 0b0001)) == ACTUATORTYPE_LIN) { // Linear Actuator
        return 1;
    } else {
        return -1;
    }
}
