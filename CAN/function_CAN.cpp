#include "function_CAN.h"
#include "setting.h"
#include "function_utilities.h"
#include "SPI_EEP_ENC.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"

// CAN ID Setting Variables
int CID_RX_CMD              = 100;
int CID_RX_REF_POSITION     = 200;
int CID_RX_REF_VALVEPOSnPWM = 300;

int CID_TX_RSP              = 1100;
int CID_TX_POS_VEL_TORQ     = 1200;
int CID_TX_VALVEPOSnPWM     = 1300;
int CID_TX_PRESSURE         = 1400;
int CID_TX_SOMETHING        = 1500;
int CID_TX_ALART            = 1600;

// variables
uint8_t can_index = 0;

extern DigitalOut LED;

extern float u_past[];
extern float x_past[];
extern float x_future[];
extern float f_past[];
extern float f_future[];
extern float input_NN[];

/*******************************************************************************
 * State Class functions
 ******************************************************************************/
void State::UpdateSen(float sen_new, float Freq_update, float f_cut) {
    if(f_cut<=0.0f) f_cut=0.001f;
    this->sen_diff = (sen_new-this->sen)*Freq_update;
    float alpha_update = 1.0f / (1.0f + Freq_update / (2.0f * 3.14f * f_cut)); 
    this->sen = (1.0f - alpha_update) * this->sen + alpha_update * sen_new;
}

void State::UpdateRef(float ref_new, float Freq_update, float f_cut) {
    if(f_cut<=0.0f) f_cut=0.001f;
    float alpha_update = 1.0f / (1.0f + Freq_update / (2.0f * 3.14f * f_cut)); 
    float ref_old = this->ref;
    this->ref = (1.0f - alpha_update) * this->ref + alpha_update * ref_new;
    this->ref_diff = (this->ref-ref_old)*Freq_update;
}

void State::Reset() {
    this->sen = 0.0f;
    this->sen_diff = 0.0f;
    this->ref = 0.0f;
    this->err = 0.0f;
    this->err_int = 0.0f;
    this->err_old = 0.0f;
    this->err_diff = 0.0f;
}

/*******************************************************************************
 * CAN functions
 ******************************************************************************/
void CAN_ID_INIT(void)
{
    CID_RX_CMD              = (int) (BNO + INIT_CID_RX_CMD);
    CID_RX_REF_POSITION     = (int) (BNO + INIT_CID_RX_REF_POSITION);
    CID_RX_REF_VALVEPOSnPWM = (int) (BNO + INIT_CID_RX_REF_VALVEPOSnPWM);

    CID_TX_RSP              = (int) (BNO + INIT_CID_TX_RSP);
    CID_TX_POS_VEL_TORQ     = (int) (BNO + INIT_CID_TX_POS_VEL_TORQ);
    CID_TX_VALVEPOSnPWM     = (int) (BNO + INIT_CID_TX_VALVEPOSnPWM);
    CID_TX_PRESSURE         = (int) (BNO + INIT_CID_TX_PRESSURE);
    CID_TX_SOMETHING        = (int) (BNO + INIT_CID_TX_SOMETHING);
    CID_TX_ALART            = (int) (BNO + INIT_CID_TX_ALART);
}

void ReadCMD(int16_t CMD)
{
    switch(CMD) {
        case CRX_ASK_INFO: {
            CAN_TX_INFO();
            break;
        }
        case CRX_ASK_BNO: {
            CAN_TX_BNO();
            break;
        }
        case CRX_SET_BNO: {
            BNO = (int16_t) msg.data[1];
            spi_eeprom_write(RID_BNO, (int16_t) BNO);
            CAN_ID_INIT(); // can id init
            break;
        }
        case CRX_ASK_OPERATING_MODE: {
            CAN_TX_OPERATING_MODE();
            break;
        }
        case CRX_SET_OPERATING_MODE: {
            OPERATING_MODE = (uint8_t) msg.data[1];
            SENSING_MODE = (uint8_t) msg.data[2];
            CURRENT_CONTROL_MODE = (uint8_t) msg.data[3];
            FLAG_VALVE_DEADZONE = (uint8_t) msg.data[4];
            spi_eeprom_write(RID_OPERATING_MODE, (int16_t) OPERATING_MODE);
            spi_eeprom_write(RID_SENSING_MODE, (int16_t) SENSING_MODE);
            spi_eeprom_write(RID_CURRENT_CONTROL_MODE, (int16_t) CURRENT_CONTROL_MODE);
            spi_eeprom_write(RID_FLAG_VALVE_DEADZONE, (int16_t) FLAG_VALVE_DEADZONE);
            break;
        }
        case CRX_SET_ENC_ZERO: {
            ENC_SET_ZERO();

            break;
        }
        case CRX_SET_FET_ON: {

            break;
        }

        case CRX_SET_POS_TORQ_TRANS: {
            MODE_POS_FT_TRANS = (int16_t) msg.data[1];
            /*
            MODE_POS_FT_TRANS == 0 : Position Control
            MODE_POS_FT_TRANS == 1 : Trasition(Position->Torque)
            MODE_POS_FT_TRANS == 2 : Torque Control (Convert to 2 automatically 3sec after transition)
            MODE_POS_FT_TRANS == 3 : Transition(Toque->Position)
            */
            break;
        }

        case CRX_ASK_CAN_FREQ: {
            CAN_TX_CAN_FREQ();

            break;
        }

        case CRX_SET_CAN_FREQ: {
            CAN_FREQ = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_CAN_FREQ, (int16_t) CAN_FREQ);
            break;
        }

        case CRX_ASK_CONTROL_MODE: {
            CAN_TX_CONTROL_MODE();

            break;
        }

        case CRX_SET_CONTROL_MODE: {
            CONTROL_UTILITY_MODE = (int16_t) (msg.data[1]);
            if (CONTROL_MODE == 22) {    //MODE_FIND_HOME
                FLAG_FIND_HOME = true;
                FINDHOME_STAGE = 0;
            }
            break;
        }

        case CRX_SET_DATA_REQUEST: {
            int request_type = msg.data[2];
            flag_data_request[request_type] = msg.data[1];
            // flag_data_request[0] : Actuator Pos & Vel & FT 
            // flag_data_request[1] : Valve Position & PWM
            // flag_data_request[2] : Chamber Pressure 
            // flag_data_request[3] : Other Something (for debugging) 
            break;
        }

        case CRX_ASK_JOINT_ENC_DIR: {
            CAN_TX_JOINT_ENC_DIR();

            break;
        }

        case CRX_SET_JOINT_ENC_DIR: {
            DIR_JOINT_ENC = (int16_t) (msg.data[1] | msg.data[2] << 8);
            if (DIR_JOINT_ENC >= 0)
                DIR_JOINT_ENC = 1;
            else
                DIR_JOINT_ENC = -1;

            spi_eeprom_write(RID_JOINT_ENC_DIR, (int16_t) DIR_JOINT_ENC);

            break;
        }

        case CRX_ASK_VALVE_DIR: {
            CAN_TX_VALVE_DIR();

            break;
        }

        case CRX_SET_VALVE_DIR: {
            DIR_VALVE = (int16_t) (msg.data[1] | msg.data[2] << 8);
            if (DIR_VALVE >= 0)
                DIR_VALVE = 1;
            else
                DIR_VALVE = -1;

            spi_eeprom_write(RID_VALVE_DIR, (int16_t) DIR_VALVE);

            break;
        }

        case CRX_ASK_VALVE_ENC_DIR: {
            CAN_TX_VALVE_ENC_DIR();

            break;
        }

        case CRX_SET_VALVE_ENC_DIR: {
            DIR_VALVE_ENC = (int16_t) (msg.data[1] | msg.data[2] << 8);
            if (DIR_VALVE_ENC >= 0)
                DIR_VALVE_ENC = 1;
            else
                DIR_VALVE_ENC = -1;

            spi_eeprom_write(RID_VALVE_ENC_DIR, (int16_t) DIR_VALVE_ENC);

            break;
        }

        case CRX_ASK_VOLTAGE_SUPPLY: {
            CAN_TX_VOLTAGE_SUPPLY();

            break;
        }

        case CRX_SET_VOLTAGE_SUPPLY: {
            SUPPLY_VOLTAGE = (double) ((int16_t) (msg.data[1] | msg.data[2] << 8)) / 10.0f;


            spi_eeprom_write(RID_VOLATGE_SUPPLY, (int16_t) (SUPPLY_VOLTAGE * 10.0f));

            break;
        }

        case CRX_ASK_VOLTAGE_VALVE: {
            CAN_TX_VOLTAGE_VALVE();

            break;
        }

        case CRX_SET_VOLTAGE_VALVE: {
            VALVE_VOLTAGE_LIMIT = (double) ((int16_t) (msg.data[1] | msg.data[2] << 8)) / 10.0f;

            spi_eeprom_write(RID_VOLTAGE_VALVE, (int16_t) (VALVE_VOLTAGE_LIMIT * 10.0f));


            break;
        }

        case CRX_SET_HOMEPOS: {
            CONTROL_UTILITY_MODE = 22;
            break;
        }
        
        case CRX_ASK_VARIABLE_SUPPLY:
        {
            CAN_TX_VARIABLE_SUPPLY_ONOFF();
            break;
        }                
        
        case CRX_SET_VARIABLE_SUPPLY:
        {
            SUPPLY_PRESSURE_UPDATE = msg.data[1];
            break;
        }

        case CRX_ASK_PID_GAIN: {
            CAN_TX_PID_GAIN(msg.data[1]);

            break;
        }

        case CRX_SET_PID_GAIN: {
            if (msg.data[1] == 0) {
                P_GAIN_VALVE_POSITION = (int16_t) (msg.data[2] | msg.data[3] << 8);
                I_GAIN_VALVE_POSITION = (int16_t) (msg.data[4] | msg.data[5] << 8);
                D_GAIN_VALVE_POSITION = (int16_t) (msg.data[6] | msg.data[7] << 8);

                spi_eeprom_write(RID_P_GAIN_VALVE_POSITION, (int16_t) P_GAIN_VALVE_POSITION);
                spi_eeprom_write(RID_I_GAIN_VALVE_POSITION, (int16_t) I_GAIN_VALVE_POSITION);
                spi_eeprom_write(RID_D_GAIN_VALVE_POSITION, (int16_t) D_GAIN_VALVE_POSITION);

            } else if (msg.data[1] == 1) {
                P_GAIN_JOINT_POSITION = (int16_t) (msg.data[2] | msg.data[3] << 8);
                I_GAIN_JOINT_POSITION = (int16_t) (msg.data[4] | msg.data[5] << 8);
                D_GAIN_JOINT_POSITION = (int16_t) (msg.data[6] | msg.data[7] << 8);

                spi_eeprom_write(RID_P_GAIN_JOINT_POSITION, (int16_t) P_GAIN_JOINT_POSITION);
                spi_eeprom_write(RID_I_GAIN_JOINT_POSITION, (int16_t) I_GAIN_JOINT_POSITION);
                spi_eeprom_write(RID_D_GAIN_JOINT_POSITION, (int16_t) D_GAIN_JOINT_POSITION);

            } else if (msg.data[1] == 2) {
                P_GAIN_JOINT_TORQUE = (int16_t) (msg.data[2] | msg.data[3] << 8);
                I_GAIN_JOINT_TORQUE = (int16_t) (msg.data[4] | msg.data[5] << 8);
                D_GAIN_JOINT_TORQUE = (int16_t) (msg.data[6] | msg.data[7] << 8);

                spi_eeprom_write(RID_P_GAIN_JOINT_TORQUE, (int16_t) P_GAIN_JOINT_TORQUE);
                spi_eeprom_write(RID_I_GAIN_JOINT_TORQUE, (int16_t) I_GAIN_JOINT_TORQUE);
                spi_eeprom_write(RID_D_GAIN_JOINT_TORQUE, (int16_t) D_GAIN_JOINT_TORQUE);

            } else if (msg.data[1] == 3) {
                K_SPRING = (float) (((float) ((int16_t) (msg.data[2] | msg.data[3] << 8))) * 0.1f);
                D_DAMPER = (float) (((float) ((int16_t) (msg.data[4] | msg.data[5] << 8))) * 0.01f);

//                spi_eeprom_write(RID_K_SPRING, (int16_t) K_SPRING);
//                spi_eeprom_write(RID_D_DAMPER, (int16_t) D_DAMPER);

            } else if (msg.data[1] == 4) {
                P_GAIN_JOINT_TORQUE_FF = (int16_t) (msg.data[2] | msg.data[3] << 8);
                I_GAIN_JOINT_TORQUE_FF = (int16_t) (msg.data[4] | msg.data[5] << 8);
                D_GAIN_JOINT_TORQUE_FF = (int16_t) (msg.data[6] | msg.data[7] << 8);

            }

            break;
        }

        case CRX_ASK_VALVE_DEADZONE: {
            CAN_TX_VALVE_DEADZONE();

            break;
        }

        case CRX_SET_VALVE_DEADZONE: {
            VALVE_CENTER = (int16_t) (msg.data[1] | msg.data[2] << 8);
            VALVE_DEADZONE_PLUS = (int16_t)(msg.data[3] | msg.data[4] << 8);
            VALVE_DEADZONE_MINUS = (int16_t)(msg.data[5] | msg.data[6] << 8);

            spi_eeprom_write(RID_VALVE_CENTER, VALVE_CENTER);
            spi_eeprom_write(RID_VALVE_DEADZONE_PLUS, VALVE_DEADZONE_PLUS);
            spi_eeprom_write(RID_VALVE_DEADZONE_MINUS, VALVE_DEADZONE_MINUS);

            break;
        }

        case CRX_ASK_VELOCITY_COMP_GAIN: {
            CAN_TX_VELOCITY_COMP_GAIN();

            break;
        }

        case CRX_SET_VELOCITY_COMP_GAIN: {
            VELOCITY_COMP_GAIN = (int16_t) (msg.data[1] | msg.data[2] << 8);


            spi_eeprom_write(RID_VELOCITY_COMP_GAIN, (int16_t) VELOCITY_COMP_GAIN);

            break;
        }

        case CRX_ASK_VALVE_ELECTRIC_CENTER: {
            CAN_TX_VALVE_ELECTRIC_CENTER();

            break;
        }

        case CRX_SET_VALVE_ELECTRIC_CENTER: {
            VALVE_ELECTRIC_CENTER = (int16_t) (msg.data[1] | msg.data[2] << 8);


            spi_eeprom_write(RID_VALVE_ELECTRIC_CENTER, (int16_t) VALVE_ELECTRIC_CENTER);

            break;
        }

        case CRX_ASK_VALVE_FF: {
            CAN_TX_VALVE_FF();

            break;
        }

        case CRX_SET_VALVE_FF: {
            VALVE_FF = (int16_t) (msg.data[1] | msg.data[2] << 8);

            spi_eeprom_write(RID_VALVE_FF, (int16_t) VALVE_FF);

            break;
        }

        case CRX_ASK_BULK_MODULUS: {
            CAN_TX_BULK_MODULUS();

            break;
        }

        case CRX_SET_BULK_MODULUS: {
            BULK_MODULUS = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_BULK_MODULUS, (int16_t) BULK_MODULUS);

            break;
        }

        case CRX_ASK_CHAMBER_VOLUME: {
            CAN_TX_CHAMBER_VOLUME();

            break;
        }

        case CRX_SET_CHAMBER_VOLUME: {
            CHAMBER_VOLUME_A = (int16_t) (msg.data[1] | msg.data[2] << 8);
            CHAMBER_VOLUME_B = (int16_t) (msg.data[3] | msg.data[4] << 8);

            spi_eeprom_write(RID_CHAMBER_VOLUME_A, (int16_t) CHAMBER_VOLUME_A);
            spi_eeprom_write(RID_CHAMBER_VOLUME_B, (int16_t) CHAMBER_VOLUME_B);

            break;
        }

        case CRX_ASK_PISTON_AREA: {
            CAN_TX_PISTON_AREA();

            break;
        }

        case CRX_SET_PISTON_AREA: {
            PISTON_AREA_A = (int16_t) (msg.data[1] | msg.data[2] << 8);
            PISTON_AREA_B = (int16_t) (msg.data[3] | msg.data[4] << 8);
            PISTON_AREA_alpha = (double)PISTON_AREA_A/(double)PISTON_AREA_B;
            alpha3 = PISTON_AREA_alpha * PISTON_AREA_alpha*PISTON_AREA_alpha;

            spi_eeprom_write(RID_PISTON_AREA_A, (int16_t) PISTON_AREA_A);
            spi_eeprom_write(RID_PISTON_AREA_B, (int16_t) PISTON_AREA_B);
            break;
        }

        case CRX_ASK_SUP_PRES: {
            CAN_TX_SUP_PRES();
            break;
        }

        case CRX_SET_SUP_PRES: {
            int16_t temp = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_PRES_SUPPLY, temp);
            PRES_SUPPLY_NOM = (float)temp;
            PRES_SUPPLY = PRES_SUPPLY_NOM;
            break;
        }

        case CRX_ASK_ENC_LIMIT: {
            CAN_TX_ENC_LIMIT();

            break;
        }

        case CRX_SET_ENC_LIMIT: {
            ENC_LIMIT_MINUS = (int16_t) (msg.data[1] | msg.data[2] << 8);
            ENC_LIMIT_PLUS = (int16_t) (msg.data[3] | msg.data[4] << 8);
            spi_eeprom_write(RID_ENC_LIMIT_MINUS, (int16_t) ENC_LIMIT_MINUS);
            spi_eeprom_write(RID_ENC_LIMIT_PLUS, (int16_t) ENC_LIMIT_PLUS);

            break;
        }

        case CRX_ASK_STROKE: {
            CAN_TX_STROKE();
            break;
        }

        case CRX_SET_STROKE: {
            STROKE = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_STROKE, (int16_t) STROKE);

            break;
        }

        case CRX_ASK_VALVE_LIMIT: {
            CAN_TX_VALVE_LIMIT();

            break;
        }

        case CRX_SET_VALVE_LIMIT: {
            VALVE_MIN_POS = (int16_t) (msg.data[1] | msg.data[2] << 8);
            VALVE_MAX_POS = (int16_t) (msg.data[3] | msg.data[4] << 8);

            spi_eeprom_write(RID_VALVE_MAX_POS, (int16_t) VALVE_MAX_POS);
            spi_eeprom_write(RID_VALVE_MIN_POS, (int16_t) VALVE_MIN_POS);

            break;
        }

        case CRX_ASK_ENC_PULSE_PER_POSITION: {
            CAN_TX_ENC_PULSE_PER_POSITION();

            break;
        }

        case CRX_SET_ENC_PULSE_PER_POSITION: {
            ENC_PULSE_PER_POSITION = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_ENC_PULSE_PER_POSITION, (int16_t) (ENC_PULSE_PER_POSITION));

            break;
        }

        case CRX_ASK_TORQUE_SENSOR_PULSE_PER_TORQUE: {
            CAN_TX_TORQUE_SENSOR_PULSE_PER_TORQUE();

            break;
        }

        case CRX_SET_TORQUE_SENSOR_PULSE_PER_TORQUE: {
//            TORQUE_SENSOR_PULSE_PER_TORQUE = (float) ((int16_t) (msg.data[1] | msg.data[2] << 8) * 0.01f);
            TORQUE_SENSOR_PULSE_PER_TORQUE = ((float) ((int16_t) (msg.data[1] | msg.data[2] << 8)))*0.001f;
            spi_eeprom_write(RID_TORQUE_SENSOR_PULSE_PER_TORQUE, (int16_t) (TORQUE_SENSOR_PULSE_PER_TORQUE*1000.0f));

            break;
        }

        case CRX_ASK_PRES_SENSOR_PULSE_PER_PRES: {
            CAN_TX_PRES_SENSOR_PULSE_PER_PRES();

            break;
        }

        case CRX_SET_PRES_SENSOR_PULSE_PER_PRES: {
            PRES_SENSOR_A_PULSE_PER_BAR = (double) ((int16_t) (msg.data[1] | msg.data[2] << 8)) * 0.01f;
            PRES_SENSOR_B_PULSE_PER_BAR = (double) ((int16_t) (msg.data[3] | msg.data[4] << 8)) * 0.01f;
            spi_eeprom_write(RID_PRES_SENSOR_A_PULSE_PER_BAR, (int16_t) (PRES_SENSOR_A_PULSE_PER_BAR * 100.0f));
            spi_eeprom_write(RID_PRES_SENSOR_B_PULSE_PER_BAR, (int16_t) (PRES_SENSOR_B_PULSE_PER_BAR * 100.0f));

            break;
        }

        case CRX_ASK_FRICTION: {
            CAN_TX_FRICTION();

            break;
        }

        case CRX_SET_FRICTION: {
            FRICTION = (double) ((int16_t) (msg.data[1] | msg.data[2] << 8)) / 10.0f;
            spi_eeprom_write(RID_FRICTION, (int16_t) (FRICTION * 10.0f));

            break;
        }

        case CRX_ASK_VALVE_GAIN_PLUS: {
            CAN_TX_VALVE_GAIN_PLUS();

            break;
        }
        case CRX_SET_VALVE_GAIN_PLUS: {
            VALVE_GAIN_LPM_PER_V[0] = (double) msg.data[1] / 50.0f;
            VALVE_GAIN_LPM_PER_V[2] = (double) msg.data[2] / 50.0f;
            VALVE_GAIN_LPM_PER_V[4] = (double) msg.data[3] / 50.0f;
            VALVE_GAIN_LPM_PER_V[6] = (double) msg.data[4] / 50.0f;
            VALVE_GAIN_LPM_PER_V[8] = (double) msg.data[5] / 50.0f;
            // spi_eeprom_write(RID_VALVE_GAIN_PLUS_1, (int16_t) (VALVE_GAIN_LPM_PER_V[0] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_PLUS_2, (int16_t) (VALVE_GAIN_LPM_PER_V[2] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_PLUS_3, (int16_t) (VALVE_GAIN_LPM_PER_V[4] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_PLUS_4, (int16_t) (VALVE_GAIN_LPM_PER_V[6] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_PLUS_5, (int16_t) (VALVE_GAIN_LPM_PER_V[8] * 100.0f));

            break;
        }

        case CRX_ASK_VALVE_GAIN_MINUS: {
            CAN_TX_VALVE_GAIN_MINUS();

            break;
        }
        case CRX_SET_VALVE_GAIN_MINUS: {
            VALVE_GAIN_LPM_PER_V[1] = (double) msg.data[1] / 50.0f;
            VALVE_GAIN_LPM_PER_V[3] = (double) msg.data[2] / 50.0f;
            VALVE_GAIN_LPM_PER_V[5] = (double) msg.data[3] / 50.0f;
            VALVE_GAIN_LPM_PER_V[7] = (double) msg.data[4] / 50.0f;
            VALVE_GAIN_LPM_PER_V[9] = (double) msg.data[5] / 50.0f;
            // spi_eeprom_write(RID_VALVE_GAIN_MINUS_1, (int16_t) (VALVE_GAIN_LPM_PER_V[1] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_MINUS_2, (int16_t) (VALVE_GAIN_LPM_PER_V[3] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_MINUS_3, (int16_t) (VALVE_GAIN_LPM_PER_V[5] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_MINUS_4, (int16_t) (VALVE_GAIN_LPM_PER_V[7] * 100.0f));
            // spi_eeprom_write(RID_VALVE_GAIN_MINUS_5, (int16_t) (VALVE_GAIN_LPM_PER_V[9] * 100.0f));

            break;
        }

        case CRX_LOW_REF: {
            REFERENCE_MODE = msg.data[1];

            REF_NUM = msg.data[2];
            REF_PERIOD = (double) ((int16_t) (msg.data[3] | msg.data[4] << 8)) / 100.0f;
            if (REF_PERIOD <= 0.0f) REF_MOVE_TIME_5k = TMR_FREQ_5k / CAN_FREQ;
            else REF_MOVE_TIME_5k = (int) (REF_PERIOD * (double) TMR_FREQ_5k);
            REF_MAG = (double) ((int16_t) (msg.data[5] | msg.data[6] << 8)) / 100.0f;

            break;
        }

        case CRX_JUMP_STATUS: {
            MODE_JUMP_STATUS = msg.data[1];

            break;
        }

        case CRX_SET_ERR_CLEAR: {
            for (int i = 0; i < 24; i++) {
                flag_alart[i] = false;
                flag_alart_old[i] = false;
            }
            break;
        }

        case CRX_ASK_HOMEPOS_OFFSET: {
            CAN_TX_HOMEPOS_OFFSET();
            break;
        }
        case CRX_SET_HOMEPOS_OFFSET: {
            HOMEPOS_OFFSET = (int16_t)(msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_HOMEPOS_OFFSET, (int16_t) HOMEPOS_OFFSET);
            break;
        }

        case CRX_ASK_HOMEPOS_VALVE_OPENING: {
            CAN_TX_HOMPOS_VALVE_OPENING();
            break;
        }
        case CRX_SET_HOMEPOS_VALVE_OPENING: {
            HOMEPOS_VALVE_OPENING = (int16_t) (msg.data[1] | msg.data[2] << 8);
            spi_eeprom_write(RID_HOMEPOS_VALVE_OPENING, (int16_t) HOMEPOS_VALVE_OPENING);
            break;
        }

        case CRX_ASK_VALVE_PWM_VS_VALVE_POS: {
            can_index = (int16_t) msg.data[1];
            CAN_TX_VALVE_PWM_VS_VALVE_POS(can_index);
            break;
        }
        case CRX_ASK_VALVE_POS_VS_FLOWRATE: {
            can_index = (int16_t) msg.data[1];
            CAN_TX_VALVE_POS_VS_FLOWRATE(can_index);
            break;
        }
        case CRX_ASK_VALVE_POS_NUM: {
            CAN_TX_VALVE_POS_NUM();
            break;
        }

        case CRX_SET_ROM: {
            break;
        }
        
        case CRX_ASK_BUFFER: {
            cnt_buffer = (int16_t) (msg.data[1] | msg.data[2] << 8);
            CAN_TX_BUFFER(cnt_buffer);
            break;
        }
        case CRX_SET_STEP_TEST: {
            cnt_step_test = 0;
            CONTROL_UTILITY_MODE = 37;
            break;
        }
        case CRX_SET_FREQ_TEST: {
            cnt_freq_test = 0;
            CONTROL_UTILITY_MODE = 34;
            break;
        }

        default:
            break;
    }
}

void CAN_RX_HANDLER()
{
    can.read(msg);
    
    unsigned int address = msg.id;
    if(address==CID_RX_CMD) {
        unsigned int CMD = msg.data[0];
        ReadCMD(CMD);

    } else if(address==CID_RX_REF_POSITION) {

        int16_t temp_pos = (int16_t) (msg.data[0] | msg.data[1] << 8); 
        int16_t temp_vel = (int16_t) (msg.data[2] | msg.data[3] << 8);
        int16_t temp_torq = (int16_t) (msg.data[4] | msg.data[5] << 8);

        if((OPERATING_MODE&0b0001)==0) { // Rotary Actuator
            pos.ref_ext = (float)temp_pos / 200.0f;
            vel.ref_ext = (float)temp_vel / 20.0f;
            torq.ref_ext = (float)temp_torq * 0.1f / TORQUE_SENSOR_PULSE_PER_TORQUE;  // pulse >> Nm
        } else { //Linear Actuator
            pos.ref_ext = (float)temp_pos / 200.0f;
            vel.ref_ext = (float)temp_vel / 20.0f;
            force.ref_ext = (float)temp_torq * 0.1f / TORQUE_SENSOR_PULSE_PER_TORQUE;  // pulse >> N
        }

        if(SUPPLY_PRESSURE_UPDATE == 1) {
            int16_t temp_REF_Ps = (int16_t) (msg.data[6] | msg.data[7] << 8);
            PRES_SUPPLY = ((float)temp_REF_Ps) / 100.0f; 
            if(PRES_SUPPLY<35.0f) PRES_SUPPLY = 35.0f;
            else if(PRES_SUPPLY>210.0f) PRES_SUPPLY = 210.0f;
        } else {
            PRES_SUPPLY = PRES_SUPPLY_NOM;
        }

    } else if(address==CID_RX_REF_VALVEPOSnPWM) {
        int16_t temp_ref_valve_pos = (int16_t) (msg.data[0] | msg.data[1] << 8);
        int16_t temp_ref_PWM = (int16_t) (msg.data[2] | msg.data[3] << 8);

        if (((OPERATING_MODE&0b0110)>>1) == 0) { //Moog Valve
            valve_pos.ref_ext = (float) temp_ref_valve_pos; // Unit : pulse (-10000~10000)
            Vout.ref_ext = (float) temp_ref_PWM;
        } else if (((OPERATING_MODE&0b0110)>>1) == 1) { //KNR Valve
            valve_pos.ref_ext = (float) temp_ref_valve_pos; // Unit : pulse (-30000~30000)
            Vout.ref_ext = (float) temp_ref_PWM; 
        } else { //SW Valve
            valve_pos.ref_ext = (float) temp_ref_valve_pos; // Unit : pulse (0~4096)
            Vout.ref_ext = (float) temp_ref_PWM; 
        }
    } 
}

/******************************************************************************
 Information Transmission Functions
*******************************************************************************/

void ALART_FLAG_ON(int _ALT_TYPE) {
    flag_alart[_ALT_TYPE] = true;
}

bool ALART_FLAG_CHECK(int _ALT_TYPE) {
    if(flag_alart[_ALT_TYPE]) 
        return true;
    else 
        return false;
}

void CAN_TX_ALART(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_ALART;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) (flag_alart[7] << 7 | flag_alart[6] << 6 | flag_alart[5] << 5 | flag_alart[4] << 4 | flag_alart[3] << 3 | flag_alart[2] << 2 | flag_alart[1] << 1 | flag_alart[0]);
    //   L [ Alart Flag 1 ]
    // flag_alart[0] : 
    // flag_alart[1] :
    // flag_alart[2] :
    // flag_alart[3] :
    // flag_alart[4] :
    // flag_alart[5] :
    // flag_alart[6] :
    // flag_alart[7] :
    temp_msg.data[1] = (uint8_t) (flag_alart[15] << 7 | flag_alart[14] << 6 | flag_alart[13] << 5 | flag_alart[12] << 4 | flag_alart[11] << 3 | flag_alart[10] << 2 | flag_alart[9] << 1 | flag_alart[8]);
    //   L [ Alart Flag 2 ]
    // flag_alart[8] : 
    // flag_alart[9] :
    // flag_alart[10] :
    // flag_alart[11] :
    // flag_alart[12] :
    // flag_alart[13] :
    // flag_alart[14] :
    // flag_alart[15] :
    temp_msg.data[2] = (uint8_t) (flag_alart[23] << 7 | flag_alart[22] << 6 | flag_alart[21] << 5 | flag_alart[20] << 4 | flag_alart[19] << 3 | flag_alart[18] << 2 | flag_alart[17] << 1 | flag_alart[16]);
    //   L [ Alart Flag 3 ]
    // flag_alart[16] : 
    // flag_alart[17] :
    // flag_alart[18] :
    // flag_alart[19] :
    // flag_alart[20] :
    // flag_alart[21] :
    // flag_alart[22] :
    // flag_alart[23] :
    can.write(temp_msg);
}

void CAN_TX_INFO(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 7;
    temp_msg.data[0] = (uint8_t) CTX_SEND_INFO;
    temp_msg.data[1] = (uint8_t) BNO;
    temp_msg.data[2] = (uint8_t) CAN_FREQ;
    temp_msg.data[3] = (uint8_t) (CAN_FREQ >> 8);
    temp_msg.data[4] = (uint8_t) (flag_alart[7] << 7 | flag_alart[6] << 6 | flag_alart[5] << 5 | flag_alart[4] << 4 | flag_alart[3] << 3 | flag_alart[2] << 2 | flag_alart[1] << 1 | flag_alart[0]);
    //   L [ Alart Flag 1 ]
    // flag_alart[0] : 
    // flag_alart[1] :
    // flag_alart[2] :
    // flag_alart[3] :
    // flag_alart[4] :
    // flag_alart[5] :
    // flag_alart[6] :
    // flag_alart[7] :
    temp_msg.data[5] = (uint8_t) (flag_alart[15] << 7 | flag_alart[14] << 6 | flag_alart[13] << 5 | flag_alart[12] << 4 | flag_alart[11] << 3 | flag_alart[10] << 2 | flag_alart[9] << 1 | flag_alart[8]);
    //   L [ Alart Flag 2 ]
    // flag_alart[8] : 
    // flag_alart[9] :
    // flag_alart[10] :
    // flag_alart[11] :
    // flag_alart[12] :
    // flag_alart[13] :
    // flag_alart[14] :
    // flag_alart[15] :
    temp_msg.data[6] = (uint8_t) (flag_alart[23] << 7 | flag_alart[22] << 6 | flag_alart[21] << 5 | flag_alart[20] << 4 | flag_alart[19] << 3 | flag_alart[18] << 2 | flag_alart[17] << 1 | flag_alart[16]);
    //   L [ Alart Flag 3 ]
    // flag_alart[16] : 
    // flag_alart[17] :
    // flag_alart[18] :
    // flag_alart[19] :
    // flag_alart[20] :
    // flag_alart[21] :
    // flag_alart[22] :
    // flag_alart[23] :
    can.write(temp_msg);
}

void CAN_TX_BNO(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    //temp_msg.len = 2;
    temp_msg.len = 2;
    temp_msg.data[0] = (uint8_t) CTX_SEND_BNO;
    temp_msg.data[1] = (uint8_t) BNO;

    can.write(temp_msg);
}

void CAN_TX_OPERATING_MODE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_OPERATING_MODE;
    temp_msg.data[1] = (uint8_t) OPERATING_MODE;
    temp_msg.data[2] = (uint8_t) SENSING_MODE;
    temp_msg.data[3] = (uint8_t) CURRENT_CONTROL_MODE;
    temp_msg.data[4] = (uint8_t) FLAG_VALVE_DEADZONE;

    can.write(temp_msg);
}

void CAN_TX_CAN_FREQ(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_CAN_FREQ;
    temp_msg.data[1] = (uint8_t) CAN_FREQ;
    temp_msg.data[2] = (uint8_t) (CAN_FREQ >> 8);

    can.write(temp_msg);
}

void CAN_TX_CONTROL_MODE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 2;
    temp_msg.data[0] = (uint8_t) CTX_SEND_CONTROL_MODE;
    temp_msg.data[1] = (uint8_t) CONTROL_UTILITY_MODE;

    can.write(temp_msg);
}

void CAN_TX_JOINT_ENC_DIR(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_JOINT_ENC_DIR;
    temp_msg.data[1] = (uint8_t) DIR_JOINT_ENC;
    temp_msg.data[2] = (uint8_t) (DIR_JOINT_ENC >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_DIR(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_DIR;
    temp_msg.data[1] = (uint8_t) DIR_VALVE;
    temp_msg.data[2] = (uint8_t) (DIR_VALVE >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_ENC_DIR(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_ENC_DIR;
    temp_msg.data[1] = (uint8_t) DIR_VALVE_ENC;
    temp_msg.data[2] = (uint8_t) (DIR_VALVE_ENC >> 8);

    can.write(temp_msg);
}

void CAN_TX_VOLTAGE_SUPPLY(void)
{
    int16_t send_voltage_supply = (int16_t) (SUPPLY_VOLTAGE * 10.0f);

    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VOLTAGE_SUPPLY;
    temp_msg.data[1] = (uint8_t) (send_voltage_supply);
    temp_msg.data[2] = (uint8_t) (send_voltage_supply >> 8);

    can.write(temp_msg);
}

void CAN_TX_VOLTAGE_VALVE(void)
{
    int16_t send_voltage_valve = (int16_t) (VALVE_VOLTAGE_LIMIT * 10.0f);

    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VOLTAGE_VALVE;
    temp_msg.data[1] = (uint8_t) send_voltage_valve;
    temp_msg.data[2] = (uint8_t) (send_voltage_valve >> 8);

    can.write(temp_msg);
}

void CAN_TX_VARIABLE_SUPPLY_ONOFF(void)
{
    CANMessage temp_msg;
    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 2;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VARIABLE_SUPPLY;
    temp_msg.data[1] = (uint8_t) SUPPLY_PRESSURE_UPDATE;

    can.write(temp_msg);
}

void CAN_TX_PID_GAIN(int t_type)
{
    // t_type = 0 : valve position control gain
    // t_type = 1 : joint position control gain
    // t_type = 2 : joint torque control gain

    int16_t sendPgain=0, sendIgain=0, sendDgain=0;
    if (t_type == 0) {
        sendPgain = (int16_t) (P_GAIN_VALVE_POSITION);
        sendIgain = (int16_t) (I_GAIN_VALVE_POSITION);
        sendDgain = (int16_t) (D_GAIN_VALVE_POSITION);
    } else if (t_type == 1) {
        sendPgain = (int16_t) (P_GAIN_JOINT_POSITION);
        sendIgain = (int16_t) (I_GAIN_JOINT_POSITION);
        sendDgain = (int16_t) (D_GAIN_JOINT_POSITION);
    } else if (t_type == 2) {
        sendPgain = (int16_t) (P_GAIN_JOINT_TORQUE);
        sendIgain = (int16_t) (I_GAIN_JOINT_TORQUE);
        sendDgain = (int16_t) (D_GAIN_JOINT_TORQUE);
    } else if (t_type == 3) {
        sendPgain = (int16_t) (K_SPRING * 10.0f);
        sendIgain = (int16_t) (D_DAMPER * 100.0f);
    }

    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 8;
    temp_msg.data[0] = (uint8_t) CTX_SEND_PID_GAIN;
    temp_msg.data[1] = (uint8_t) t_type;
    temp_msg.data[2] = (uint8_t) sendPgain;
    temp_msg.data[3] = (uint8_t) (sendPgain >> 8);
    temp_msg.data[4] = (uint8_t) sendIgain;
    temp_msg.data[5] = (uint8_t) (sendIgain >> 8);
    temp_msg.data[6] = (uint8_t) sendDgain;
    temp_msg.data[7] = (uint8_t) (sendDgain >> 8);

    can.write(temp_msg);
}


void CAN_TX_VALVE_DEADZONE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 7;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_DEADZONE;
    temp_msg.data[1] = (uint8_t) VALVE_CENTER;
    temp_msg.data[2] = (uint8_t) (VALVE_CENTER >> 8);
    temp_msg.data[3] = (uint8_t) VALVE_DEADZONE_PLUS;
    temp_msg.data[4] = (uint8_t) (VALVE_DEADZONE_PLUS >> 8);
    temp_msg.data[5] = (uint8_t) VALVE_DEADZONE_MINUS;
    temp_msg.data[6] = (uint8_t) (VALVE_DEADZONE_MINUS >> 8);

    can.write(temp_msg);
}

void CAN_TX_VELOCITY_COMP_GAIN(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VELOCITY_COMP_GAIN;
    temp_msg.data[1] = (uint8_t) VELOCITY_COMP_GAIN;
    temp_msg.data[2] = (uint8_t) (VELOCITY_COMP_GAIN >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_ELECTRIC_CENTER(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_ELECTRIC_CENTER;
    temp_msg.data[1] = (uint8_t) VALVE_ELECTRIC_CENTER;
    temp_msg.data[2] = (uint8_t) (VALVE_ELECTRIC_CENTER >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_FF(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_FF;
    temp_msg.data[1] = (uint8_t) VALVE_FF;
    temp_msg.data[2] = (uint8_t) (VALVE_FF >> 8);

    can.write(temp_msg);
}

void CAN_TX_BULK_MODULUS(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_BULK_MODULUS;
    temp_msg.data[1] = (uint8_t) BULK_MODULUS;
    temp_msg.data[2] = (uint8_t) (BULK_MODULUS >> 8);

    can.write(temp_msg);
}

void CAN_TX_CHAMBER_VOLUME(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_CHAMBER_VOLUME;
    temp_msg.data[1] = (uint8_t) CHAMBER_VOLUME_A;
    temp_msg.data[2] = (uint8_t) (CHAMBER_VOLUME_A >> 8);
    temp_msg.data[3] = (uint8_t) CHAMBER_VOLUME_B;
    temp_msg.data[4] = (uint8_t) (CHAMBER_VOLUME_B >> 8);

    can.write(temp_msg);
}

void CAN_TX_PISTON_AREA(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_PISTON_AREA;
    temp_msg.data[1] = (uint8_t) PISTON_AREA_A;
    temp_msg.data[2] = (uint8_t) (PISTON_AREA_A >> 8);
    temp_msg.data[3] = (uint8_t) PISTON_AREA_B;
    temp_msg.data[4] = (uint8_t) (PISTON_AREA_B >> 8);

    can.write(temp_msg);
}

void CAN_TX_SUP_PRES(void)
{
    CANMessage temp_msg;

    int16_t temp_PRES_SUPPLY = (int16_t) (PRES_SUPPLY);
    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_SUP_PRES;
    temp_msg.data[1] = (uint8_t) temp_PRES_SUPPLY;
    temp_msg.data[2] = (uint8_t) (temp_PRES_SUPPLY >> 8);
    temp_msg.data[3] = 0;
    temp_msg.data[4] = 0;

    can.write(temp_msg);
}

void CAN_TX_ENC_LIMIT(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_ENC_LIMIT;
    temp_msg.data[1] = (uint8_t) ENC_LIMIT_MINUS;
    temp_msg.data[2] = (uint8_t) (ENC_LIMIT_MINUS >> 8);
    temp_msg.data[3] = (uint8_t) ENC_LIMIT_PLUS;
    temp_msg.data[4] = (uint8_t) (ENC_LIMIT_PLUS >> 8);

    can.write(temp_msg);
}

void CAN_TX_STROKE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_STROKE;
    temp_msg.data[1] = (uint8_t) STROKE;
    temp_msg.data[2] = (uint8_t) (STROKE >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_LIMIT(void)
{
    CANMessage temp_msg;
     int16_t temp_valve_min_pos = 0;
     int16_t temp_valve_max_pos = 0;
     temp_valve_min_pos = (int16_t) VALVE_MIN_POS;
     temp_valve_max_pos = (int16_t) VALVE_MAX_POS;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_LIMIT;
    temp_msg.data[1] = (uint8_t) temp_valve_min_pos;
    temp_msg.data[2] = (uint8_t) (temp_valve_min_pos >> 8);
    temp_msg.data[3] = (uint8_t) temp_valve_max_pos;
    temp_msg.data[4] = (uint8_t) (temp_valve_max_pos >> 8);

    can.write(temp_msg);
}

void CAN_TX_ENC_PULSE_PER_POSITION(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_ENC_PULSE_PER_POSITION;
    int temp_enc_pulse_per_position = (int) (ENC_PULSE_PER_POSITION);
    temp_msg.data[1] = (uint8_t) temp_enc_pulse_per_position;
    temp_msg.data[2] = (uint8_t) (temp_enc_pulse_per_position >> 8);

    can.write(temp_msg);
}

void CAN_TX_TORQUE_SENSOR_PULSE_PER_TORQUE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_TORQUE_SENSOR_PULSE_PER_TORQUE;
    int16_t temp_torque_sensor_pulse_per_torque = (int16_t) (TORQUE_SENSOR_PULSE_PER_TORQUE * 1000.0f);
    temp_msg.data[1] = (uint8_t) temp_torque_sensor_pulse_per_torque;
    temp_msg.data[2] = (uint8_t) (temp_torque_sensor_pulse_per_torque >> 8);

    can.write(temp_msg);
}

void CAN_TX_PRES_SENSOR_PULSE_PER_PRES(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_PRES_SENSOR_PULSE_PER_BAR;
    temp_msg.data[1] = (uint8_t) (int) (PRES_SENSOR_A_PULSE_PER_BAR * 100.0f);
    temp_msg.data[2] = (uint8_t) ((int) (PRES_SENSOR_A_PULSE_PER_BAR * 100.0f) >> 8);
    temp_msg.data[3] = (uint8_t) (int) (PRES_SENSOR_B_PULSE_PER_BAR * 100.0f);
    temp_msg.data[4] = (uint8_t) ((int) (PRES_SENSOR_B_PULSE_PER_BAR * 100.0f) >> 8);

    can.write(temp_msg);
}

void CAN_TX_FRICTION(void)
{
    CANMessage temp_msg;
    int16_t send_friction;
    send_friction = (int16_t) (FRICTION * 10.0f);

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_FRICTION;
    temp_msg.data[1] = (uint8_t) send_friction;
    temp_msg.data[2] = (uint8_t) (send_friction >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_GAIN_PLUS(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_GAIN_PLUS;
    temp_msg.data[1] = (uint8_t) (VALVE_GAIN_LPM_PER_V[0] * 50.0f);
    temp_msg.data[2] = (uint8_t) (VALVE_GAIN_LPM_PER_V[2] * 50.0f);
    temp_msg.data[3] = (uint8_t) (VALVE_GAIN_LPM_PER_V[4] * 50.0f);
    temp_msg.data[4] = (uint8_t) (VALVE_GAIN_LPM_PER_V[6] * 50.0f);
    temp_msg.data[5] = (uint8_t) (VALVE_GAIN_LPM_PER_V[8] * 50.0f);

    can.write(temp_msg);
}


void CAN_TX_VALVE_GAIN_MINUS(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) CTX_SEND_VALVE_GAIN_MINUS;
    temp_msg.data[1] = (uint8_t) (VALVE_GAIN_LPM_PER_V[1] * 50.0f);
    temp_msg.data[2] = (uint8_t) (VALVE_GAIN_LPM_PER_V[3] * 50.0f);
    temp_msg.data[3] = (uint8_t) (VALVE_GAIN_LPM_PER_V[5] * 50.0f);
    temp_msg.data[4] = (uint8_t) (VALVE_GAIN_LPM_PER_V[7] * 50.0f);
    temp_msg.data[5] = (uint8_t) (VALVE_GAIN_LPM_PER_V[9] * 50.0f);

    can.write(temp_msg);
}

void CAN_TX_REFENCE_MODE(void)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) CTX_SEND_REFENCE_MODE;
    temp_msg.data[1] = (uint8_t) REFERENCE_MODE;
    temp_msg.data[2] = (uint8_t) (int) (REFERENCE_FREQ * 100.0f);
    temp_msg.data[3] = (uint8_t) ((int) (REFERENCE_FREQ * 100.0f) >> 8);
    temp_msg.data[4] = (uint8_t) (int) (REFERENCE_MAG * 100.0f);
    temp_msg.data[5] = (uint8_t) ((int) (REFERENCE_MAG * 100.0f) >> 8);

    can.write(temp_msg);
}

void CAN_TX_HOMEPOS_OFFSET(void)
{
    CANMessage temp_msg;
    int16_t send_homepos_offset;
    send_homepos_offset = (int16_t)HOMEPOS_OFFSET;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_HOMEPOS_OFFSET;
    temp_msg.data[1] = (uint8_t) send_homepos_offset;
    temp_msg.data[2] = (uint8_t) (send_homepos_offset >> 8);

    can.write(temp_msg);
}

void CAN_TX_HOMPOS_VALVE_OPENING(void)
{
    CANMessage temp_msg;
    int16_t send_homepos_valve_opening;
    send_homepos_valve_opening = (int16_t) (HOMEPOS_VALVE_OPENING);

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_SEND_HOMEPOS_VALVE_OPENING;
    temp_msg.data[1] = (uint8_t) send_homepos_valve_opening;
    temp_msg.data[2] = (uint8_t) (send_homepos_valve_opening >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVE_PWM_VS_VALVE_POS(int8_t canindex)
{
    CANMessage temp_msg;
    
    int16_t PWM_VALVE_ID = (int16_t)(PWMs_for_ValvePosID_Sorted[canindex] * 1000.0f);
    int16_t valve_pos_vs_pwm = (int16_t)(VALVE_POS_VS_PWM_Sorted[canindex]); // Unit : pulse 

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) CTX_VALVE_PWM_VS_VALVE_POS;
    temp_msg.data[1] = (uint8_t) PWM_VALVE_ID;
    temp_msg.data[2] = (uint8_t) (PWM_VALVE_ID >> 8);
    temp_msg.data[3] = (uint8_t) valve_pos_vs_pwm;
    temp_msg.data[4] = (uint8_t) (valve_pos_vs_pwm >> 8);
    temp_msg.data[5] = canindex;

    can.write(temp_msg);
}

void CAN_TX_VALVE_POS_VS_FLOWRATE(int8_t canindex)
{
    CANMessage temp_msg;

    float VALVE_POS_ID_f;
    if(ValvePosOff_for_FlowrateID_Sorted[canindex]>=0) {
        VALVE_POS_ID_f = ValvePosOff_for_FlowrateID_Sorted[canindex]/(float)((float)VALVE_MAX_POS-(float)VALVE_CENTER)*100.0f;
    } else {
        VALVE_POS_ID_f = ValvePosOff_for_FlowrateID_Sorted[canindex]/(float)((float)VALVE_CENTER-(float)VALVE_MIN_POS)*100.0f;
    }

    if(VALVE_POS_ID_f>100.0f) VALVE_POS_ID_f = 100.0f;
    if(VALVE_POS_ID_f<-100.0f) VALVE_POS_ID_f = -100.0f;

    int16_t VALVE_POS_ID = (int16_t)(VALVE_POS_ID_f*100.0f);
    int16_t valve_pos_vs_flowrate = (int16_t)(FlowRate_VS_ValvePosOff_Sorted[canindex]*1000.0f);

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) CTX_VALVE_POS_VS_FLOWRATE;
    temp_msg.data[1] = (uint8_t) VALVE_POS_ID;
    temp_msg.data[2] = (uint8_t) (VALVE_POS_ID >> 8);
    temp_msg.data[3] = (uint8_t) valve_pos_vs_flowrate;
    temp_msg.data[4] = (uint8_t) (valve_pos_vs_flowrate >> 8);
    temp_msg.data[5] = canindex;

    can.write(temp_msg);
}

void CAN_TX_VALVE_POS_NUM(void)
{
    CANMessage temp_msg;
    int32_t valve_pos_num;
    valve_pos_num = (int16_t) VALVE_POS_NUM;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 3;
    temp_msg.data[0] = (uint8_t) CTX_VALVE_POS_NUM;
    temp_msg.data[1] = (uint8_t) valve_pos_num;
    temp_msg.data[2] = (uint8_t) (valve_pos_num >> 8);

    can.write(temp_msg);
}

void CAN_TX_DDV_VALVE_MAX_MIN_POS(void)
{
    CANMessage temp_msg;

    float temp_valve_max_pos = 0.0f;
    float temp_valve_min_pos = 0.0f;
    float temp_ddv_center = 0.0f;

    temp_valve_max_pos = VALVE_MAX_POS;
    temp_valve_min_pos = VALVE_MIN_POS;
    temp_ddv_center = VALVE_CENTER;

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 7;
    temp_msg.data[0] = (uint8_t) CTX_VALVE_MAX_MIN_POS;
    temp_msg.data[1] = (uint8_t) temp_valve_max_pos;
    temp_msg.data[2] = (uint8_t) ((int) (temp_valve_max_pos) >> 8);
    temp_msg.data[3] = (uint8_t) (temp_valve_min_pos);
    temp_msg.data[4] = (uint8_t) ((int) (temp_valve_min_pos) >> 8);
    temp_msg.data[5] = (uint8_t) (temp_ddv_center);
    temp_msg.data[6] = (uint8_t) ((int) (temp_ddv_center) >> 8);

    can.write(temp_msg);
}


void CAN_TX_BUFFER(int16_t t_cnt_buffer)
{
    CANMessage temp_msg;
    int16_t send_pos_array, send_ref_array;
    send_pos_array = (int16_t) (pos_array[t_cnt_buffer]);
    send_ref_array = (int16_t) (ref_array[t_cnt_buffer]);

    temp_msg.id = CID_TX_RSP;
    temp_msg.len = 5;
    temp_msg.data[0] = (uint8_t) CTX_SEND_BUFFER;
    temp_msg.data[1] = (uint8_t) send_pos_array;
    temp_msg.data[2] = (uint8_t) (send_pos_array >> 8);
    temp_msg.data[3] = (uint8_t) (send_ref_array);
    temp_msg.data[4] = (uint8_t) ((send_ref_array) >> 8);

    can.write(temp_msg);
}


/******************************************************************************
 Sensor & State Transmission Functions
*******************************************************************************/

void CAN_TX_POSnFT(int16_t t_pos, int16_t t_vel, int16_t t_torq)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_POS_VEL_TORQ;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) t_pos;
    temp_msg.data[1] = (uint8_t) (t_pos >> 8);
    temp_msg.data[2] = (uint8_t) t_vel;
    temp_msg.data[3] = (uint8_t) (t_vel >> 8);
    temp_msg.data[4] = (uint8_t) t_torq;
    temp_msg.data[5] = (uint8_t) (t_torq >> 8);

    can.write(temp_msg);
}

void CAN_TX_VALVEPOSnPWM(int16_t t_valve_pos, int16_t t_ref_valve_pos, int16_t t_ref_PWM)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_VALVEPOSnPWM;
    temp_msg.len = 6;
    temp_msg.data[0] = (uint8_t) t_valve_pos;
    temp_msg.data[1] = (uint8_t) (t_valve_pos >> 8);
    temp_msg.data[2] = (uint8_t) t_ref_valve_pos;
    temp_msg.data[3] = (uint8_t) (t_ref_valve_pos >> 8);
    temp_msg.data[4] = (uint8_t) t_ref_PWM;
    temp_msg.data[5] = (uint8_t) (t_ref_PWM >> 8);

    can.write(temp_msg);
}

void CAN_TX_PRESSURE(int16_t t_pres_a, int16_t t_pres_b)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_PRESSURE;
    temp_msg.len = 4;
    temp_msg.data[0] = (uint8_t) t_pres_a;
    temp_msg.data[1] = (uint8_t) (t_pres_a >> 8);
    temp_msg.data[2] = (uint8_t) t_pres_b;
    temp_msg.data[3] = (uint8_t) (t_pres_b >> 8);

    can.write(temp_msg);
}

void CAN_TX_SOMETHING(int16_t t_a, int16_t t_b, int16_t t_c, int16_t t_d)
{
    CANMessage temp_msg;

    temp_msg.id = CID_TX_SOMETHING;
    temp_msg.len = 8;
    temp_msg.data[0] = (uint8_t) t_a;
    temp_msg.data[1] = (uint8_t) (t_a >> 8);
    temp_msg.data[2] = (uint8_t) t_b;
    temp_msg.data[3] = (uint8_t) (t_b >> 8);
    temp_msg.data[4] = (uint8_t) t_c;
    temp_msg.data[5] = (uint8_t) (t_c >> 8);
    temp_msg.data[6] = (uint8_t) t_d;
    temp_msg.data[7] = (uint8_t) (t_d >> 8);

    can.write(temp_msg);
}

