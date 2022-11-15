//Hydraulic Control Board
//distributed by Sungwoo Kim
//       2020/12/28
//revised by Buyoun Cho
//       2022/10/06

// [Commented @ 221006]
// 이 코드는 DDV용으로 사용하지 않음.
// Moog사 30 series 제어용으로만 사용.

// 유의사항
// 소수 적을때 뒤에 f 꼭 붙이기
// CAN 선은 ground까지 있는 3상 선으로 써야함.
// 전원은 12~24V 인가.

#include "mbed.h"
#include "FastPWM.h"
#include "INIT_HW.h"
#include "function_CAN.h"
#include "SPI_EEP_ENC.h"
#include "I2C_AS5510.h"
#include "setting.h"
#include "function_utilities.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include <string>
#include <iostream>
#include <cmath>

using namespace std;
Timer t;

// dac & check ///////////////////////////////////////////
DigitalOut check(PC_2);
DigitalOut check_2(PC_3);
AnalogOut dac_1(PA_4); // 0.0f ~ 1.0f
AnalogOut dac_2(PA_5); // 0.0f ~ 1.0f
AnalogIn adc1(PC_4); //pressure_1
AnalogIn adc2(PB_0); //pressure_2
AnalogIn adc3(PC_1); //current

// PWM ///////////////////////////////////////////
float dtc_v=0.0f;
float dtc_w=0.0f;

// I2C ///////////////////////////////////////////
I2C i2c(PC_9,PA_8); // SDA, SCL (for K22F)
const int i2c_slave_addr1 =  0x56;  // AS5510 address
unsigned int value; // 10bit output of reading sensor AS5510

// SPI ///////////////////////////////////////////
SPI eeprom(PB_15, PB_14, PB_13); // EEPROM //(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut eeprom_cs(PB_12);
SPI enc(PC_12,PC_11,PC_10);
DigitalOut enc_cs(PD_2);
DigitalOut LED(PA_15);

// // UART ///////////////////////////////////////////
// Serial pc(PA_9,PA_10); //  _ UART

// CAN ///////////////////////////////////////////
CAN can(PB_8, PB_9, 1000000);
CANMessage msg;
void onMsgReceived()
{
    CAN_RX_HANDLER();
}

// State Variables ///////////////////////////////////////////
State pos;
State vel;
State force;
State torq;         // unit : N
State torq_dot;
State pres_A;       // unit : bar
State pres_B;
State cur;          // unit : mA
State valve_pos;
State valve_pos_raw;
State Vout;

// CAN ID  ///////////////////////////////////////////
extern int CID_RX_CMD;
extern int CID_RX_REF_POSITION;
extern int CID_RX_REF_VALVEPOS;
extern int CID_RX_REF_PWM;

extern int CID_TX_RSP;
extern int CID_TX_POS_VEL_TORQ;
extern int CID_TX_VALVE_POSITION;
extern int CID_TX_PRESSURE;
extern int CID_TX_PWMnCURRENT;
extern int CID_TX_SOMETHING;

inline float tanh_inv(float y)
{
    if(y >= 1.0f - 0.000001f) y = 1.0f - 0.000001f;
    if(y <= -1.0f + 0.000001f) y = -1.0f + 0.000001f;
    return log(sqrt((1.0f+y)/(1.0f-y)));
}

/*******************************************************************************
 * ENUMARATED TYPES
 ******************************************************************************/

enum _REFERENCE_MODE {
    MODE_REF_NO_ACT = 0,
    MODE_REF_EXTERNAL,
    MODE_REF_UTILMODE
};

enum _CONTROL_MODE {
    //control mode
    MODE_NO_ACT = 0,             // 0
    MODE_JOINT_CONTROL,          // 1
    MODE_VALVE_POSITION_CONTROL, // 2
    MODE_VALVE_OPEN_LOOP,        // 3 

    //utility
    MODE_TORQUE_SENSOR_NULLING   = 20,                    
    MODE_FIND_HOME               = 22,                                     

    MODE_ID_VALVEPOS_VS_PWM      = 30,                      
    MODE_ID_DEADZONE_AND_CENTER  = 31,                 
    MODE_ID_FLOWRATE_VS_VALVEPOS = 32,                       

};

/*******************************************************************************
 Untility Mode Functions and Variables 
 ******************************************************************************/
float Mapping_ValvePos2PWM(float _REF_VALVE_POS);
float Mapping_FlowRate2ValvePos(float _REF_FLOWRATE);
float VALVE_POS_CONTROL(float REF_VALVE_POS);

int CNT4UtilityMode = 0;
int CNT4UtilityMode_Old = 0;
float UtilityMode_Pos = 0.0f;
float UtilityMode_PosOld = 0.0f;
float UtilityMode_PosRef = 0.0f;
float UtilityMode_PosRef_INIT = 0.0f;
float UtilityMode_Vel = 0.0f;
int UtilityMode_ID_index = 0;

enum _FINDHOME_STAGESET {
    FINDHOME_INIT = 0,
    FINDHOME_GOTOLIMIT,
    FINDHOME_ZEROPOSE
};
int FINDHOME_STAGE = FINDHOME_INIT;
bool Flag_FINDHOME_DONE = false;
int cnt_touch_end = 0;
bool Flag_PosMidFound = false;

enum _VALVEPOS_VS_PWM_ID_STAGESET {
    VALVEPOS_VS_PWM_ID_INIT = 0,
    VALVEPOS_VS_PWM_ID_MAIN,
    VALVEPOS_VS_PWM_ID_TERMINATE
};
int STAGE_VALVEPOS_VS_PWM_ID = VALVEPOS_VS_PWM_ID_INIT;

enum _VALVEDZ_ID_STAGESET {
    VALVEDZ_ID_INIT = 0,
    VALVEDZ_ID_FIND_POS_PLUS,
    VALVEDZ_ID_FIND_POS_MINUS,
    VALVEDZ_ID_MOVE2MID,
    VALVEDZ_ID_FIND_STARTPOINT,
    VALVEDZ_ID_FIND_DZBAND_LOWERBOUND,
    VALVEDZ_ID_FIND_DZBAND_UPPERBOUND,
    VALVEDZ_ID_TERMINATE
};
int STAGE_VALVEDZ_ID = VALVEDZ_ID_INIT;

enum _FLOWRATE_VS_VALVEPOS_ID_STAGESET {
    FLOWRATE_VS_VALVEPOS_ID_INIT = 0,
    FLOWRATE_VS_VALVEPOS_ID_FIND_POS_PLUS,
    FLOWRATE_VS_VALVEPOS_ID_FIND_POS_MINUS,
    FLOWRATE_VS_VALVEPOS_ID_MOVE2MID,
    FLOWRATE_VS_VALVEPOS_ID_MAIN,
    FLOWRATE_VS_VALVEPOS_ID_RETURN2MID,
    FLOWRATE_VS_VALVEPOS_ID_TERMINATE
};
int STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_INIT;

float PosCtrl4UtilFunc(float _PosRef);
void UtilFunc_TORQUE_SENSOR_NULLING(void);
void UtilFunc_FIND_HOME(void);
void UtilFunc_ID_VALVEPOS_VS_PWM(void);
void UtilFunc_ID_DEADZONE_AND_CENTER(void);
void UtilFunc_ID_FLOWRATE_VS_VALVEPOS(void);

//////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;//8
    RCC_OscInitStruct.PLL.PLLN = 180; //180
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        //Error_Handler();
    }
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        //Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        //Error_Handler();
    }
}


int main()
{
    /*********************************
    ***     Initialization
    *********************************/

    HAL_Init();
    SystemClock_Config();

    LED = 0;
    // pc.baud(9600);

//    // i2c init
//    i2c.frequency(400 * 1000);          // 0.4 mHz
//    wait_ms(2);                         // Power Up wait
//    look_for_hardware_i2c();            // Hardware present
//    init_as5510(i2c_slave_addr1);
//    make_delay();

    // spi init
    eeprom_cs = 1;
    eeprom.format(8,3);
    eeprom.frequency(5000000); //5M
    eeprom_cs = 0;
    make_delay();

    enc_cs = 1;     //sw add
    enc.format(8,0);
    enc.frequency(5000000); //10M
    enc_cs = 0;     //sw add

    make_delay();

    // spi _ enc
    spi_enc_set_init();
    make_delay();

    ////// bno rom
    spi_eeprom_write(RID_BNO, (int16_t) 8);
    spi_eeprom_write(RID_OPERATING_MODE, (int16_t)1); // MOOG TSV & Linear Actuator
    // spi_eeprom_write(RID_OPERATING_MODE, (int16_t)0); // MOOG TSV & Rotary Actuator
    make_delay();
    ////////

    // rom
    ROM_CALL_DATA();
    make_delay();


    // Parameter Setting according to the Valve Type
    if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) {
        VALVE_MAX_POS = 10000;
        VALVE_MIN_POS = -10000;
        DZ_OPENSIZE = 512.0f;
        INIT_DZ_OPENSIZE = 512.0f;

        PWMs_for_ValvePosID[0] = 0.0f;
        PWMs_for_ValvePosID[1] = 200.0f;
        PWMs_for_ValvePosID[2] = -200.0f;
        PWMs_for_ValvePosID[3] = 300.0f;
        PWMs_for_ValvePosID[4] = -300.0f;
        PWMs_for_ValvePosID[5] = 400.0f;
        PWMs_for_ValvePosID[6] = -400.0f;
        PWMs_for_ValvePosID[7] = 500.0f;
        PWMs_for_ValvePosID[8] = -500.0f;
        PWMs_for_ValvePosID[9] = 600.0f;
        PWMs_for_ValvePosID[10] = -600.0f;
        PWMs_for_ValvePosID[11] = 700.0f;
        PWMs_for_ValvePosID[12] = -700.0f;
        PWMs_for_ValvePosID[13] = 800.0f;
        PWMs_for_ValvePosID[14] = -800.0f;
        PWMs_for_ValvePosID[15] = 1000.0f;
        PWMs_for_ValvePosID[16] = -1000.0f;
        PWMs_for_ValvePosID[17] = 1200.0f;
        PWMs_for_ValvePosID[18] = -1200.0f;
        PWMs_for_ValvePosID[19] = 1400.0f;
        PWMs_for_ValvePosID[20] = -1400.0f;
        PWMs_for_ValvePosID[21] = 1700.0f;
        PWMs_for_ValvePosID[22] = -1700.0f;
        PWMs_for_ValvePosID[23] = 2000.0f;
        PWMs_for_ValvePosID[24] = -2000.0f;
        PWMs_for_ValvePosID[25] = 2500.0f;
        PWMs_for_ValvePosID[26] = -2500.0f;
        PWMs_for_ValvePosID[27] = 3000.0f;
        PWMs_for_ValvePosID[28] = -3000.0f;
        PWMs_for_ValvePosID[29] = 3500.0f;
        PWMs_for_ValvePosID[30] = -3500.0f;
        PWMs_for_ValvePosID[31] = 4000.0f;
        PWMs_for_ValvePosID[32] = -4000.0f;
        PWMs_for_ValvePosID[33] = 4500.0f;
        PWMs_for_ValvePosID[34] = -4500.0f;
        PWMs_for_ValvePosID[35] = 5000.0f;
        PWMs_for_ValvePosID[36] = -5000.0f;
        PWMs_for_ValvePosID[37] = 5500.0f;
        PWMs_for_ValvePosID[38] = -5500.0f;
        PWMs_for_ValvePosID[39] = 6000.0f;
        PWMs_for_ValvePosID[40] = -6000.0f;
        for (int i = 0; i < NUM_VALVEPOS_VS_PWM_ID; i++) {
            PWMs_for_ValvePosID[i] = 0.001f*PWMs_for_ValvePosID[i];
            if (i % 2 == 0) {
                // PWMs_for_ValvePosID[i] = -((float)i * 0.5f * 0.350f);
                PWMs_for_ValvePosID_Sorted[(NUM_VALVEPOS_VS_PWM_ID-1-i)/2] = PWMs_for_ValvePosID[i];
            } else {
                // PWMs_for_ValvePosID[i] = (float)(i+1) * 0.5f * 0.350f;
                PWMs_for_ValvePosID_Sorted[(NUM_VALVEPOS_VS_PWM_ID+i)/2] = PWMs_for_ValvePosID[i];
            }
        }

        ValvePosOff_for_FlowrateID[0] = 0.0f;
        ValvePosOff_for_FlowrateID[1] = 40.0f;
        ValvePosOff_for_FlowrateID[2] = -40.0f;
        ValvePosOff_for_FlowrateID[3] = 80.0f;
        ValvePosOff_for_FlowrateID[4] = -80.0f;
        ValvePosOff_for_FlowrateID[5] = 120.0f;
        ValvePosOff_for_FlowrateID[6] = -120.0f;
        ValvePosOff_for_FlowrateID[7] = 160.0f;
        ValvePosOff_for_FlowrateID[8] = -160.0f;
        ValvePosOff_for_FlowrateID[9] = 200.0f;
        ValvePosOff_for_FlowrateID[10] = -200.0f;
        ValvePosOff_for_FlowrateID[11] = 300.0f;
        ValvePosOff_for_FlowrateID[12] = -300.0f;
        ValvePosOff_for_FlowrateID[13] = 400.0f;
        ValvePosOff_for_FlowrateID[14] = -400.0f;
        ValvePosOff_for_FlowrateID[15] = 500.0f;
        ValvePosOff_for_FlowrateID[16] = -500.0f;
        ValvePosOff_for_FlowrateID[17] = 700.0f;
        ValvePosOff_for_FlowrateID[18] = -700.0f;
        ValvePosOff_for_FlowrateID[19] = 900.0f;
        ValvePosOff_for_FlowrateID[20] = -900.0f;
        ValvePosOff_for_FlowrateID[21] = 1100.0f;
        ValvePosOff_for_FlowrateID[22] = -1100.0f;
        ValvePosOff_for_FlowrateID[23] = 1400.0f;
        ValvePosOff_for_FlowrateID[24] = -1400.0f;
        ValvePosOff_for_FlowrateID[25] = 1700.0f;
        ValvePosOff_for_FlowrateID[26] = -1700.0f;
        ValvePosOff_for_FlowrateID[27] = 2000.0f;
        ValvePosOff_for_FlowrateID[28] = -2000.0f;
        ValvePosOff_for_FlowrateID[29] = 2500.0f;
        ValvePosOff_for_FlowrateID[30] = -2500.0f;
        ValvePosOff_for_FlowrateID[31] = 3000.0f;
        ValvePosOff_for_FlowrateID[32] = -3000.0f;
        ValvePosOff_for_FlowrateID[33] = 3500.0f;
        ValvePosOff_for_FlowrateID[34] = -3500.0f;
        ValvePosOff_for_FlowrateID[35] = 4000.0f;
        ValvePosOff_for_FlowrateID[36] = -4000.0f;
        ValvePosOff_for_FlowrateID[37] = 5000.0f;
        ValvePosOff_for_FlowrateID[38] = -5000.0f;
        ValvePosOff_for_FlowrateID[39] = 6000.0f;
        ValvePosOff_for_FlowrateID[40] = -6000.0f;
        ValvePosOff_for_FlowrateID[41] = 7000.0f;
        ValvePosOff_for_FlowrateID[42] = -7000.0f;
        ValvePosOff_for_FlowrateID[43] = 8000.0f;
        ValvePosOff_for_FlowrateID[44] = -8000.0f;
        ValvePosOff_for_FlowrateID[45] = 8500.0f;
        ValvePosOff_for_FlowrateID[46] = -8500.0f;
        ValvePosOff_for_FlowrateID[47] = 9000.0f;
        ValvePosOff_for_FlowrateID[48] = -9000.0f;
        ValvePosOff_for_FlowrateID[49] = 9500.0f;
        ValvePosOff_for_FlowrateID[50] = -9500.0f;
        for (int i = 0; i < NUM_FLOWRATE_VS_VALVEPOS_ID; i++) {
            if (i % 2 == 0) {
                // ValvePosOff_for_FlowrateID[i] = -((float)i * 0.5f * 400.0f);
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID-1-i)/2] = ValvePosOff_for_FlowrateID[i];
            } else {
                // ValvePosOff_for_FlowrateID[i] = (float)(i+1) * 0.5f * 400.0f;
                ValvePosOff_for_FlowrateID_Sorted[(NUM_FLOWRATE_VS_VALVEPOS_ID+i)/2] = ValvePosOff_for_FlowrateID[i];
            }
        }
    }

    // ADC init
    Init_ADC();
    make_delay();

    // Pwm init
    Init_PWM();
    TIM4->CR1 ^= TIM_CR1_UDIS;
    make_delay();

    // CAN
    can.attach(&CAN_RX_HANDLER);
    CAN_ID_INIT();
    make_delay();

    //can.reset();
    can.filter(msg.id, 0xFFFFF000, CANStandard);

    // TMR3 init
    Init_TMR3();
    TIM3->CR1 ^= TIM_CR1_UDIS;
    make_delay();

    //Timer priority
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_SetPriority(TIM4_IRQn, 3);

    //DAC init
    if (SENSING_MODE == 0) {
        dac_1 = FORCE_VREF / 3.3f;
        dac_2 = 0.0f;
    } else if (SENSING_MODE == 1) {
        if (DIR_VALVE_ENC > 0) {
            dac_1 = PRES_A_VREF / 3.3f;
            dac_2 = PRES_B_VREF / 3.3f;
        } else {
            dac_1 = PRES_B_VREF / 3.3f;
            dac_2 = PRES_A_VREF / 3.3f;
        }
    } else if (SENSING_MODE == 2) {
        dac_1 = 0.0f;
        dac_2 = FORCE_B_VREF / 3.3f;
    }
    make_delay();

    /************************************
    ***     Program is operating!
    *************************************/
    while(1) {

        // UART example
//        if(timer_while==100000) {
//            timer_while = 0;
//            pc.printf("%f\n", value);
//        }
//        timer_while ++;

        //i2c for SW valve
        //if(OPERATING_MODE == 5) {
//            read_field(i2c_slave_addr1);
//            if(DIR_VALVE_ENC < 0) value = 1023 - value;
//        }
    }
}


// Main End
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*******************************************************************************
                            TIMER INTERRUPT
*******************************************************************************/

//------------------------------------------------
//     TMR4 : Sensor Read & Data Handling
//-----------------------------------------------
float FREQ_TMR4 = (float)FREQ_20k;
float DT_TMR4 = (float)DT_20k;
long  CNT_TMR4 = 0;
int   TMR4_FREQ_10k = (int)FREQ_10k;
extern "C" void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF ) {

        // Current ===================================================
        // ADC3->CR2  |= 0x40000000;                        // adc _ 12bit
        valve_pos.UpdateSen(((float)ADC3->DR-2047.5f)/2047.5f*10000.0f, FREQ_TMR4, 1000.0f); // unit : mA
        cur.UpdateSen(((float)ADC3->DR-2047.5f)/2047.5f*10.0f, FREQ_TMR4, 1000.0f); // unit : mA

        // Encoder ===================================================
        if (CNT_TMR4 % (int) ((int) FREQ_TMR4/TMR4_FREQ_10k) == 0) {
            ENC_UPDATE();
        }

        // Force or Pressure Transducer =============================================
        ADC1->CR2  |= 0x40000000;
        if (SENSING_MODE == 0) {  // Force sensing
            force.UpdateSen((((float)ADC1->DR) - 2047.5f)/TORQUE_SENSOR_PULSE_PER_TORQUE, FREQ_TMR4, 200.0f); // unit : N
        } else if (SENSING_MODE == 1) { // Pressure sensing
            float pres_A_new, pres_B_new;
            if (DIR_VALVE_ENC > 0) {
                pres_A_new = (((float)ADC1->DR) - PRES_A_NULL_pulse)/ PRES_SENSOR_A_PULSE_PER_BAR; // unit : bar
                pres_B_new = (((float)ADC2->DR) - PRES_B_NULL_pulse)/ PRES_SENSOR_B_PULSE_PER_BAR;
            } else {
                pres_A_new = (((float)ADC2->DR) - PRES_A_NULL_pulse)/ PRES_SENSOR_A_PULSE_PER_BAR; // unit : bar
                pres_B_new = (((float)ADC1->DR) - PRES_B_NULL_pulse)/ PRES_SENSOR_B_PULSE_PER_BAR;
            }
            pres_A.UpdateSen(pres_A_new,FREQ_TMR4,200.0f);
            pres_B.UpdateSen(pres_B_new,FREQ_TMR4,200.0f);

            if ((OPERATING_MODE & 0b01) == 0) { // Rotary Actuator
                float torq_new = (PISTON_AREA_A * pres_A.sen - PISTON_AREA_B * pres_B.sen) * 0.0001f; // mm^3*bar >> Nm
                torq.UpdateSen(torq_new,FREQ_TMR4,200.0f);  // unit : Nm
            } else if ((OPERATING_MODE & 0b01) == 1) { // Linear Actuator
                float force_new = (PISTON_AREA_A * pres_A.sen - PISTON_AREA_B * pres_B.sen) * 0.1f; // mm^2*bar >> N
                force.UpdateSen(force_new,FREQ_TMR4,200.0f);  // unit : N
            }
        } else if (SENSING_MODE == 2) {  // Force sensing with B port
            force.UpdateSen((((float)ADC2->DR) - 2047.5f)/TORQUE_SENSOR_PULSE_PER_TORQUE, FREQ_TMR4, 200.0f); // unit : N
        }

        CNT_TMR4++;
    }
    TIM4->SR = 0x0;  // reset the status register
}

//------------------------------------------------
//     TMR3 : Control 5kHz
//------------------------------------------------
float FREQ_TMR3 = (float)FREQ_5k;
float DT_TMR3 = (float)DT_5k;
int cnt_trans = 0;
float force_ref_act_can = 0.0f;
int MODE_LED_Display = 0;  

extern "C" void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF ) {

        ////////////////////////////////////////////////////////////////////
        ///////////////////// Frequency Checking LED ///////////////////////
        //////////////////////////////////////////////////////////////////// 
        if(MODE_LED_Display == 0) {
            static int CNT_LED1 = 0;
            if(CNT_LED1 % (int)(FREQ_5k*1.0f) == 0) {  // Period = 1.0 sec
                CNT_LED1 = 0;
                LED = !LED;
            } CNT_LED1++;
        } else if (MODE_LED_Display == 1) {
            static int CNT_LED2 = 0;
            if(CNT_LED2 % (int)(FREQ_5k*0.25f) == 0) { // Period = 0.25 sec
                CNT_LED2 = 0;
                LED = !LED;
            } CNT_LED2++;
        } else if (MODE_LED_Display == 2) { 
            static int CNT_LED3 = 0;
            if(CNT_LED3 % (int)(FREQ_5k*4.0f) == 0) {  // Period = 4.0 sec
                CNT_LED3 = 0;
                LED = !LED;
            } CNT_LED3++;
        } else if (MODE_LED_Display == 3) {
            LED = 1;
        } else if (MODE_LED_Display == 4) {
            LED = 0;
        } else {
            LED = 0;
        }

        // Valve parameter setting
        if (Classify_ValveType() == VALVETYPE_MOOG) {       // Moog two-stage valve parameters
            K_v = 0.90f; // Q = K_v*sqrt(deltaP)*tanh(C_d*Xv);
            C_d = 0.14f;
            mV_PER_mA = 500.0f;    // 5000mV/10mA
            mV_PER_pulse = 0.5f;   // 5000mV/10000pulse
            mA_PER_pulse = 0.001f; // 10mA/10000pulse
        } else if (Classify_ValveType() == VALVETYPE_KNR) {          // KNR two-stage valve parameters
            K_v = 0.5f;            // KNR (LPM >> mA) , 100bar
            mV_PER_mA = 166.6666f; // 5000mV/30mA
            mV_PER_pulse = 0.5f;   // 5000mV/10000pulse
            mA_PER_pulse = 0.003f; // 30mA/10000pulse
        }

        // =====================================================================
        // CONTROL MODE CLASSIFICATION -----------------------------------------
        // =====================================================================
        int UTILITY_MODE = 0;
        int CONTROL_MODE = 0;

        if (CONTROL_UTILITY_MODE >= 20) {
            UTILITY_MODE = CONTROL_UTILITY_MODE;
            CONTROL_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_UTILMODE;
        } else if (CONTROL_UTILITY_MODE == 0) {
           UTILITY_MODE = MODE_NO_ACT;
            CONTROL_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_NO_ACT;
        } else {
            CONTROL_MODE = CONTROL_UTILITY_MODE;
            UTILITY_MODE = MODE_NO_ACT;
            REFERENCE_MODE = MODE_REF_EXTERNAL;
        }

        ////////////////////////////////////////////////////////////////////
        //////// Reference Update (Selecting External or Internal) /////////
        ////////////////////////////////////////////////////////////////////       
        switch (REFERENCE_MODE) {
            case MODE_REF_NO_ACT: {
                break;
            }
            case MODE_REF_EXTERNAL: {
                pos.UpdateRef(pos.ref_ext, FREQ_5k, 200.0f);
                vel.UpdateRef(vel.ref_ext, FREQ_5k, 200.0f);
                torq.UpdateRef(torq.ref_ext, FREQ_5k, 200.0f);
                force.UpdateRef(force.ref_ext, FREQ_5k, 200.0f);
                valve_pos.UpdateRef(valve_pos.ref_ext, FREQ_5k, 200.0f);
                Vout.UpdateRef(Vout.ref_ext, FREQ_5k, 200.0f);
                break;
            }
            case MODE_REF_UTILMODE: {
                pos.ref = UtilityMode_PosRef;
                vel.ref = 0.0f;
                torq.ref = 0.0f;
                force.ref = 0.0f;
                break;
            }
            default:
                break;
        }

        ////////////////////////////////////////////////////////////////////
        /////////////////// UTILITY MODE ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////
        switch (UTILITY_MODE) {
        case MODE_NO_ACT: 
            break;
        case MODE_TORQUE_SENSOR_NULLING: 
            UtilFunc_TORQUE_SENSOR_NULLING();
            break;
        case MODE_FIND_HOME: 
            CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
            UtilFunc_FIND_HOME();
            break;
        case MODE_ID_VALVEPOS_VS_PWM: 
            CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
            UtilFunc_ID_VALVEPOS_VS_PWM();
            break;        
        case MODE_ID_DEADZONE_AND_CENTER: 
            CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
            UtilFunc_ID_DEADZONE_AND_CENTER();
            break;
        case MODE_ID_FLOWRATE_VS_VALVEPOS: 
            CONTROL_MODE = MODE_VALVE_OPEN_LOOP;
            UtilFunc_ID_FLOWRATE_VS_VALVEPOS();
            break;
        default:
            break;
        }

        ////////////////////////////////////////////////////////////////////
        /////////////////// CONTROL MODE ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////

        switch (CONTROL_MODE) {
        case MODE_NO_ACT: {
            V_out = 0.0f;
            break;
        }
        case MODE_JOINT_CONTROL: {

            // ===============================================================================================================================================
            // Pos. or Force Control Mode Parameter Setting 
            if (MODE_POS_FT_TRANS == 1) { // position control >> force control mode transition
                if (alpha_trans == 1.0f) MODE_POS_FT_TRANS = 2;
                alpha_trans = (float)(1.0f - cos(3.141592f * (float)cnt_trans * DT_TMR3 / 3.0f)) / 2.0f;
                cnt_trans++;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                pos.err_int = 0.0f;
                if ((float)cnt_trans * DT_TMR3 > 3.0f) {
                    MODE_POS_FT_TRANS = 2;
                }
            } else if (MODE_POS_FT_TRANS == 3) { // force control >> position control mode transition
                if (alpha_trans == 0.0f) MODE_POS_FT_TRANS = 0;
                alpha_trans = (float)(1.0f + cos(3.141592f * (float)cnt_trans * DT_TMR3 / 3.0f)) / 2.0f;
                cnt_trans++;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                pos.err_int = 0.0f;
                if ((float)cnt_trans * DT_TMR3 > 3.0f) {
                    MODE_POS_FT_TRANS = 0;
                }
            } else if (MODE_POS_FT_TRANS == 2) { // force control mode
                alpha_trans = 1.0f;
                pos.err_int = 0.0f;
                cnt_trans = 0;
            } else { // MODE_POS_FT_TRANS == 0, position control mode
                alpha_trans = 0.0f;
                torq.err_int = 0.0f;
                force.err_int = 0.0f;
                cnt_trans = 0;
            }

            float temp_vel_pos = 0.0f; // desired velocity for position feedback control
            float temp_vel_FT = 0.0f;  // desired velocity for force/torque feedback control
            float temp_vel_ff = 0.0f;  // desired velocity for velocity feedforward control
            float temp_vel = 0.0f; // summation of all terms above

            // ===============================================================================================================================================
            // position feedback control command
            float wn_Pos = 2.0f * PI * 5.0f; // f_cut : 5Hz Position Control
            pos.err = pos.ref - pos.sen; // Unit : mm or deg
            vel.err = vel.ref - vel.sen; // Unit : mm/s or deg/s
            pos.err_int += pos.err*DT_5k;       
            float Kp_POS, Ki_POS;
            temp_vel_pos = wn_Pos * pos.err;
            if(fabs(temp_vel_pos)<=4.0f) { // under 5mm/s or 5deg/s
                Kp_POS = 3.0f * P_GAIN_JOINT_POSITION; 
                // Ki_POS = 2.0f * I_GAIN_JOINT_POSITION;
            } else if (fabs(temp_vel_pos)<=8.0f) {
                Kp_POS = (1.0f + 2.0f*(8.0f-fabs(temp_vel_pos))/4.0f) * P_GAIN_JOINT_POSITION; 
                // Ki_POS = (1.0f + 1.0f*(6.0f-fabs(temp_vel_pos))/3.0f) * I_GAIN_JOINT_POSITION;
            } else {
                Kp_POS = 1.0f * P_GAIN_JOINT_POSITION; 
                // Ki_POS = 1.0f * I_GAIN_JOINT_POSITION;
            }
            // Kp_POS = 1.0f * P_GAIN_JOINT_POSITION; 
            Ki_POS = 1.0f * I_GAIN_JOINT_POSITION;

            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                temp_vel_pos = 0.01f * wn_Pos * (Kp_POS * pos.err + Ki_POS * pos.err_int) * PI / 180.0f; // rad/s
                //                        L when P-gain = 100, f_cut = 5Hz
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 10.0f));
                pos.err_int = (1.0f-alpha_int)*pos.err_int;
            } else { // Linear Mode
                temp_vel_pos = 0.01f * wn_Pos * (Kp_POS * pos.err + Ki_POS * pos.err_int); // mm/s
                //                        L when P-gain = 100, f_cut = 5Hz
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 10.0f));
                pos.err_int = (1.0f-alpha_int)*pos.err_int;
            }

            // ===============================================================================================================================================
            // torque feedback control command
            float alpha_SpringDamper = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 30.0f));
            K_LPF = (1.0f - alpha_SpringDamper) * K_LPF + alpha_SpringDamper * K_SPRING;
            D_LPF = (1.0f - alpha_SpringDamper) * D_LPF + alpha_SpringDamper * D_DAMPER;

            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                float torq_ref_act = torq.ref + K_LPF * pos.err + D_LPF * vel.err; // unit : Nm
                if(torq_ref_act>PRES_SUPPLY*(float)PISTON_AREA_A*0.0001f*0.9f) torq_ref_act = PRES_SUPPLY*(float)PISTON_AREA_A*0.0001f*0.9f;
                if(torq_ref_act<-PRES_SUPPLY*(float)PISTON_AREA_B*0.0001f*0.9f) torq_ref_act = -PRES_SUPPLY*(float)PISTON_AREA_B*0.0001f*0.9f;

                torq.err = torq_ref_act - torq.sen;
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                torq.err_int = (1.0f-alpha_int)*torq.err_int;
                torq.err_int += torq.err / ((float)TMR_FREQ_5k);
                // if (torq.err > 5.0f || torq.err < -5.0f) {
                //     torq.err_int += torq.err / ((float)TMR_FREQ_5k);
                // }
                if(I_GAIN_JOINT_TORQUE*force.err_int>3000.0f) torq.err_int = 3000.0f/I_GAIN_JOINT_TORQUE;
                if(I_GAIN_JOINT_TORQUE*force.err_int<-3000.0f) torq.err_int = -3000.0f/I_GAIN_JOINT_TORQUE;
                temp_vel_FT = 0.001f * (P_GAIN_JOINT_TORQUE * torq.err + I_GAIN_JOINT_TORQUE * torq.err_int); // Nm >> rad/s
            } else { // Linear Mode
                float force_ref_act = force.ref + K_LPF * pos.err + D_LPF * vel.err; // unit : N
                if(force_ref_act>PRES_SUPPLY*(float)PISTON_AREA_A*0.1f*0.9f) force_ref_act = PRES_SUPPLY*(float)PISTON_AREA_A*0.1f*0.9f;
                if(force_ref_act<-PRES_SUPPLY*(float)PISTON_AREA_B*0.1f*0.9f) force_ref_act = -PRES_SUPPLY*(float)PISTON_AREA_B*0.1f*0.9f;

                force.err = force_ref_act - force.sen;
                float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                force.err_int = (1.0f-alpha_int)*force.err_int;
                // force.err_int += force.err / ((float)TMR_FREQ_5k);
                if (force.err < 20.0f && force.err > -20.0f) {
                    force.err = 0.0f;
                } else {
                    force.err_int += force.err / ((float)TMR_FREQ_5k);
                }
                if(I_GAIN_JOINT_TORQUE*force.err_int>30000.0f) force.err_int = 30000.0f/I_GAIN_JOINT_TORQUE;
                if(I_GAIN_JOINT_TORQUE*force.err_int<-30000.0f) force.err_int = -30000.0f/I_GAIN_JOINT_TORQUE;
                temp_vel_FT = 0.01f * (P_GAIN_JOINT_TORQUE * force.err + I_GAIN_JOINT_TORQUE * force.err_int); // N >> mm/s
            }

            // ========================================================================================================================================
            // velocity feedforward command
            if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
                temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref * PI / 180.0f; // rad/s
            } else { // Linear Mode
                temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref; // mm/s
            }

            // =================================================================================================================================================
            // command integration
            temp_vel = (1.0f - alpha_trans) * temp_vel_pos + // Position Control
                        alpha_trans * temp_vel_FT + // + Torque Control
                        temp_vel_ff; //  + Velocity Feedforward

            float Qact = 0.0f; // required flow rate
            float ValvePos_JC = 0.0f; // Valve Position Ref for Joint Control
            if (temp_vel > 0.0f) {
                Qact = temp_vel * ((float)PISTON_AREA_A * 0.00006f); // mm^3/sec >> LPM
                if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                    ValvePos_JC = tanh_inv(Qact /(K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)))) / C_d*1000.0f;
                    // ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                } else { // SW valve
                    // ValvePos_JC =  (float)VALVE_CENTER + Qact / (C_d * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)));
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                }
            } else {
                Qact = temp_vel * ((float)PISTON_AREA_B * 0.00006f); // mm^3/sec >> LPM
                if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
                    ValvePos_JC = tanh_inv(Qact / (K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)))) / C_d*1000.0f;
                    // ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                  
                } else { // SW valve
                    // ValvePos_JC = (float)VALVE_CENTER + Qact / (C_d * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)));
                    ValvePos_JC =  Mapping_FlowRate2ValvePos(Qact);                   
                }
            }

            if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve                
                // Anti-windup for FT
                if (I_GAIN_JOINT_TORQUE > 0.001f) {
                    float Ka = 0.1f;
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        float ValvePos_rem = ValvePos_JC - (float)VALVE_MAX_POS;
                        ValvePos_JC = (float)VALVE_MAX_POS;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)) * tanh(C_d * ValvePos_rem) / ((float)PISTON_AREA_A * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        float ValvePos_rem = ValvePos_JC - (float)VALVE_MIN_POS;
                        ValvePos_JC = (float)VALVE_MIN_POS;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)) * tanh(C_d * ValvePos_rem) / ((float)PISTON_AREA_B * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f / I_GAIN_JOINT_TORQUE);
                    }
                } else {
                    if (ValvePos_JC > (float)VALVE_MAX_POS) {
                        ValvePos_JC = (float)VALVE_MAX_POS;
                    } else if (ValvePos_JC < (float)VALVE_MIN_POS) {
                        ValvePos_JC = (float)VALVE_MIN_POS;
                    }
                }
                V_out = VALVE_POS_CONTROL(ValvePos_JC);
            } 
            break;
        }
        case MODE_VALVE_POSITION_CONTROL: {
            // Two-stage Valve : ValvePosition = ValveInputCurrent (-10000.0f ~ 10000.0f)
            // Direct Drive Valve : ValvePosition = LVDT Value (0.0f ~ 4095.0f)
            V_out = VALVE_POS_CONTROL(valve_pos.ref);
            break;
        }
        
        case MODE_VALVE_OPEN_LOOP: {
            V_out = Vout.ref;
            break;
        }

        default:
            break;
        }

        ////////////////////////////////////////////////////////////////////
        ///////////////////  PWM Command ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////
        if(DIR_VALVE<0) {
            V_out = -V_out;
        }

        // //  Dead Zone Cancellation & Linearization 
        // V_out = Mapping_OutputVoltage2PWMCommand_MoogTSV(V_out);

        if (V_out >= VALVE_VOLTAGE_LIMIT * 1000.0f) {
            V_out = VALVE_VOLTAGE_LIMIT * 1000.0f;
        } else if (V_out <= -VALVE_VOLTAGE_LIMIT * 1000.0f) {
            V_out = -VALVE_VOLTAGE_LIMIT * 1000.0f;
        }

        PWM_out = V_out / (SUPPLY_VOLTAGE * 1000.0f); // Unit : -1.0 ~ 1.0

        // Saturation of output voltage
        if (PWM_out > 1.0f)
            PWM_out = 1.0f;
        else if (PWM_out < -1.0f)
            PWM_out = -1.0f;

        if (PWM_out>0.0f) {
            dtc_v=0.0f;
            dtc_w=PWM_out;
        } else {
            dtc_v=-PWM_out;
            dtc_w=0.0f;
        }

        //pwm
        TIM4->CCR2 = (PWM_ARR)*(1.0f-dtc_v);
        TIM4->CCR1 = (PWM_ARR)*(1.0f-dtc_w);

        ////////////////////////////////////////////////////////////////////////////
        //////////////////////  Data transmission through CAN //////////////////////
        ////////////////////////////////////////////////////////////////////////////

        if (TMR2_COUNT_CAN_TX % (int) ((int) TMR_FREQ_5k/CAN_FREQ) == 0) {

            // Position, Velocity, and Torque (ID:1200)
            if (flag_data_request[0] == HIGH) {
                if ((OPERATING_MODE & 0b01) == 0) { // Rotary Actuator
                    CAN_TX_POSnFT((int16_t) (pos.sen*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (torq.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
                } else if ((OPERATING_MODE & 0b01) == 1) { // Linear Actuator
                    CAN_TX_POSnFT((int16_t) (pos.sen*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (force.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
                }
                // CAN_TX_POSnFT(1201,1202,1203);
            }

            // Valve Position (ID:1300)
            if (flag_data_request[1]) {
                CAN_TX_VALVEPOSnPWM((int16_t)(valve_pos.sen), (int16_t)(valve_pos.ref), (int16_t)(V_out));
                // CAN_TX_VALVEPOSnPWM((int16_t)(valve_pos.sen), (int16_t)(cur.sen), (int16_t)(V_out));
                // CAN_TX_VALVEPOSnPWM(1301,1302,1303);
            }

            // Pressure (ID:1400)
            if (flag_data_request[2]) {
                CAN_TX_PRESSURE((int16_t)(pres_A.sen*100.0f), (int16_t)(pres_B.sen*100.0f));
                // CAN_TX_PRESSURE(1401,1402);
            }

            // Something for debugging (ID:1500)
            if (flag_data_request[3]) {
                // CAN_TX_SOMETHING((int16_t)DebugVar_Float[0],(int16_t)DebugVar_Float[1],(int16_t)DebugVar_Int[0],(int16_t)DebugVar_Int[1]);
                CAN_TX_SOMETHING((int16_t)DebugVar_Float[0],(int16_t)DebugVar_Float[1],(int16_t)DebugVar_Float[2],(int16_t)DebugVar_Float[3]);
                // CAN_TX_SOMETHING(1501,1501,1502,1503);
            }

            TMR2_COUNT_CAN_TX = 0;
        }
        TMR2_COUNT_CAN_TX++;

    }
    TIM3->SR = 0x0;  // reset the status register

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ======================================= Utility Functions  ==================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float Mapping_ValvePos2PWM(float _REF_VALVE_POS) {
    float _PWM = 0.0;
    
    if(_REF_VALVE_POS < VALVE_POS_VS_PWM_Sorted[0]) {
        _PWM = PWMs_for_ValvePosID_Sorted[0]*1000.0f;
        return _PWM;
    }

    if(_REF_VALVE_POS > VALVE_POS_VS_PWM_Sorted[NUM_VALVEPOS_VS_PWM_ID-1]) {
        _PWM = PWMs_for_ValvePosID_Sorted[NUM_VALVEPOS_VS_PWM_ID-1]*1000.0f;
        return _PWM;
    }

    for (int i = 0; i < (NUM_VALVEPOS_VS_PWM_ID-1); i++) {
        if (_REF_VALVE_POS >= VALVE_POS_VS_PWM_Sorted[i] && _REF_VALVE_POS <= VALVE_POS_VS_PWM_Sorted[i+1] ) {
            _PWM = PWMs_for_ValvePosID_Sorted[i] + (PWMs_for_ValvePosID_Sorted[i+1]-PWMs_for_ValvePosID_Sorted[i])*(_REF_VALVE_POS-VALVE_POS_VS_PWM_Sorted[i])/(VALVE_POS_VS_PWM_Sorted[i+1]-VALVE_POS_VS_PWM_Sorted[i]);
            _PWM = _PWM * 1000.0f;
            return _PWM;
        }
    }

    return 0.0f;
}

float Mapping_FlowRate2ValvePos(float _REF_FLOWRATE) {

    float _Ps_ID = 100.0; // Table was obtained at Ps = 100 bar & double rod (No Load) 
    float _Ps = PRES_SUPPLY;
    float _a3 = alpha3;

    float _REF_FLOWRATE_Scaled = 0.0f;
    if(_REF_FLOWRATE > 0.0f) {
        _REF_FLOWRATE_Scaled = sqrt((_a3+1.0f)/(2.0f*_a3)*(_Ps_ID/_Ps))*_REF_FLOWRATE;
    } else {
        _REF_FLOWRATE_Scaled = sqrt((_a3+1.0f)/(2.0f)*(_Ps_ID/_Ps))*_REF_FLOWRATE;
    }

    float _REF_VALVE_POS_OFF = 0.0f;
    float _REF_VALVE_POS = 0.0f;
    
    float FlowRate_VS_ValvePosOff_Masked[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f,};
    
    // float mask[3] = {0.0f,1.0f,0.0f};
    // ArrayMasking_1st(FlowRate_VS_ValvePosOff_Sorted, FlowRate_VS_ValvePosOff_Masked, NUM_FLOWRATE_VS_VALVEPOS_ID, mask);
    float mask[5] = {0.5f,0.0f,0.0f,0.0f,0.5f};
    ArrayMasking_2nd(FlowRate_VS_ValvePosOff_Sorted, FlowRate_VS_ValvePosOff_Masked, NUM_FLOWRATE_VS_VALVEPOS_ID, mask);

    if(_REF_FLOWRATE_Scaled < FlowRate_VS_ValvePosOff_Masked[0]) {
        _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[0];
        _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
        return _REF_VALVE_POS;
    }
    if(_REF_FLOWRATE_Scaled > FlowRate_VS_ValvePosOff_Masked[NUM_FLOWRATE_VS_VALVEPOS_ID-1]) {
        _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[NUM_FLOWRATE_VS_VALVEPOS_ID-1];
        _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
        return _REF_VALVE_POS;
    }

    for (int i = 0; i < (NUM_FLOWRATE_VS_VALVEPOS_ID-1); i++) {
        if (_REF_FLOWRATE_Scaled >= FlowRate_VS_ValvePosOff_Masked[i] && _REF_FLOWRATE_Scaled <= FlowRate_VS_ValvePosOff_Masked[i+1] ) {
            _REF_VALVE_POS_OFF = ValvePosOff_for_FlowrateID_Sorted[i] + (ValvePosOff_for_FlowrateID_Sorted[i+1]-ValvePosOff_for_FlowrateID_Sorted[i])*(_REF_FLOWRATE_Scaled-FlowRate_VS_ValvePosOff_Masked[i])/(FlowRate_VS_ValvePosOff_Masked[i+1]-FlowRate_VS_ValvePosOff_Masked[i]);
            _REF_VALVE_POS = (float)VALVE_CENTER + _REF_VALVE_POS_OFF;
            return _REF_VALVE_POS;
        }
    }
}

float VALVE_POS_CONTROL(float REF_VALVE_POS) {
    
    static float REF_VALVE_POS_OLD = 0.0f;
    float _Vout = 0.0f;

    if (REF_VALVE_POS > (float)VALVE_MAX_POS) {
        REF_VALVE_POS = (float)VALVE_MAX_POS;
    } else if (REF_VALVE_POS < (float)VALVE_MIN_POS) {
        REF_VALVE_POS = (float)VALVE_MIN_POS;
    }

    // Valve Spool Position Control for Two-Stage Valve (TSV)
    if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) {

        // Moog Valve Current Control Gain
        float R_model = 500.0f; // ohm
        float L_model = 1.2f;
        float w0 = 2.0f * 3.14f * 70.0f;
        float KP_I = 1.0f * L_model * w0;
        float KI_I = 1.0f * R_model * w0;
        // KNR Valve Current Control Gain
        if (Classify_ValveType() == VALVETYPE_KNR) { // KNR Valve
            R_model = 163.0f; // ohm
            L_model = 1.0f;
            w0 = 2.0f * 3.14f * 80.0f;
            KP_I = 1.0f * L_model * w0;
            KI_I = 0.08f * R_model * w0;
        }

        VALVE_PWM_RAW_FF = 1.0f*Mapping_ValvePos2PWM(REF_VALVE_POS); // Unit : mV
        // VALVE_PWM_RAW_FF = 1.0f*(R_model*(REF_VALVE_POS/1000.0f)); // Unit : mV

        valve_pos_err = (float)(REF_VALVE_POS - valve_pos.sen)/1000.0f; // Unit : mA
        valve_pos_err_sum += valve_pos_err*DT_5k;
        if (valve_pos_err_sum > 1000.0f) valve_pos_err_sum = 1000.0f;
        if (valve_pos_err_sum < -1000.0f) valve_pos_err_sum = -1000.0f;

        VALVE_PWM_RAW_FB = KP_I * valve_pos_err + KI_I * valve_pos_err_sum; // Unit : mV

        _Vout = VALVE_PWM_RAW_FF + VALVE_PWM_RAW_FB; // Unit : mV
        // _Vout = VALVE_PWM_RAW_FF; // Unit : mV
        // _Vout = VALVE_PWM_RAW_FB; // Unit : mV

        float V_MAX = VALVE_VOLTAGE_LIMIT * 1000.0f; // Maximum Voltage, Unit : mV
        float V_rem = 0.0f;
        float Ka = 1.0f;
        if (_Vout > V_MAX) {
            V_rem = Ka * (_Vout - V_MAX);
            _Vout = V_MAX;
            valve_pos_err_sum = valve_pos_err_sum - V_rem / KP_I;
        } else if (_Vout < -V_MAX) {
            V_rem = Ka * (_Vout - (-V_MAX));
            _Vout = -V_MAX;
            valve_pos_err_sum = valve_pos_err_sum - V_rem / KP_I;
        }
    }

    return _Vout;
}


float PosCtrl4UtilFunc(float _PosRef)
{
    float temp_vel_UtilFunc = 0.0f;
    float wn_Pos = 2.0f * PI * 3.0f; // f_cut : 5Hz Position Control
    if (Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Mode
        temp_vel_UtilFunc =  wn_Pos * (_PosRef - pos.sen) * PI / 180.0f; // deg/s >> rad/s
    } else { // Linear Mode
        temp_vel_UtilFunc =  wn_Pos * (_PosRef - pos.sen); // mm/s
    }

    float Qact_UtilFunc = 0.0f;
    float ValvePos_UtilFunc = 0.0f;
    if (temp_vel_UtilFunc > 0.0f) {
        Qact_UtilFunc = temp_vel_UtilFunc * ((float)PISTON_AREA_A * 0.00006f); // mm^3/sec >> LPM
        if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
            ValvePos_UtilFunc = tanh_inv(Qact_UtilFunc /(K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)))) / C_d*1000.0f;
        } else { // SW valve
            ValvePos_UtilFunc =  (float)VALVE_CENTER + Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)));
        }
    } else {
        Qact_UtilFunc = temp_vel_UtilFunc * ((float)PISTON_AREA_B * 0.00006f); // mm^3/sec >> LPM
        if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
            ValvePos_UtilFunc = tanh_inv(Qact_UtilFunc / (K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)))) / C_d*1000.0f;
        } else { // SW valve
            ValvePos_UtilFunc = (float)VALVE_CENTER + Qact_UtilFunc / (C_d * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)));
        }
    }

    if (ValvePos_UtilFunc > (float)VALVE_MAX_POS) {
        ValvePos_UtilFunc = (float)VALVE_MAX_POS;
    } else if (ValvePos_UtilFunc < (float)VALVE_MIN_POS) {
        ValvePos_UtilFunc = (float)VALVE_MIN_POS;
    }

    DebugVar_Float[0] = ValvePos_UtilFunc;

    return VALVE_POS_CONTROL(ValvePos_UtilFunc);

    // if (Classify_ValveType()==VALVETYPE_MOOG || Classify_ValveType()==VALVETYPE_KNR) { // Moog Valve or KNR Valve
    //     return (Qact_UtilFunc/5.0f*2000.0f);  // Output : mV               
    // } else { // SW valve
    //     return (Qact_UtilFunc/5.0f*6000.0f);  // Output : mV                  
    // }
}

void UtilFunc_TORQUE_SENSOR_NULLING(void) 
{
    static float FORCE_pulse_sum = 0.0f;
    static float PresA_pulse_sum = 0.0f;
    static float PresB_pulse_sum = 0.0f;

    // DAC Voltage reference set
    float VREF_TuningGain = -0.000003f;
    if (TMR3_COUNT_TORQUE_NULL < TMR_FREQ_5k * 5) {
        if (SENSING_MODE == 0) { // Force Sensor (Loadcell)
            FORCE_pulse_sum = FORCE_pulse_sum + force.sen * TORQUE_SENSOR_PULSE_PER_TORQUE;
            if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                float FORCE_pluse_mean = FORCE_pulse_sum / 10.0f;
                FORCE_pulse_sum = 0.0f;
                FORCE_VREF += VREF_TuningGain * (0.0f - FORCE_pluse_mean);
                if (FORCE_VREF > 3.3f) FORCE_VREF = 3.3f;
                if (FORCE_VREF < 0.0f) FORCE_VREF = 0.0f;
                dac_1 = FORCE_VREF / 3.3f;
            }
        } else if (SENSING_MODE == 1) { // Pressure Sensor
            PresA_pulse_sum += pres_A.sen * PRES_SENSOR_A_PULSE_PER_BAR;
            PresB_pulse_sum += pres_B.sen * PRES_SENSOR_B_PULSE_PER_BAR;
            if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                float PresA_pluse_mean = PresA_pulse_sum / 10.0f;
                float PresB_pluse_mean = PresB_pulse_sum / 10.0f;
                PresA_pulse_sum = 0.0f;
                PresB_pulse_sum = 0.0f;

                PRES_A_VREF += VREF_TuningGain * (0.0f - PresA_pluse_mean);
                if (PRES_A_VREF > 3.3f) PRES_A_VREF = 3.3f;
                if (PRES_A_VREF < 0.0f) PRES_A_VREF = 0.0f;
                dac_1 = PRES_A_VREF / 3.3f;
                PRES_B_VREF += VREF_TuningGain * (0.0f - PresB_pluse_mean);
                if (PRES_B_VREF > 3.3f) PRES_B_VREF = 3.3f;
                if (PRES_B_VREF < 0.0f) PRES_B_VREF = 0.0f;
                dac_2 = PRES_B_VREF / 3.3f;
            }
        } else if (SENSING_MODE == 2) {
            FORCE_pulse_sum = FORCE_pulse_sum + force.sen * TORQUE_SENSOR_PULSE_PER_TORQUE;
            if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                float FORCE_pluse_mean = FORCE_pulse_sum / 10.0f;
                FORCE_pulse_sum = 0.0f;
                FORCE_B_VREF += VREF_TuningGain * (0.0f - FORCE_pluse_mean);
                if (FORCE_B_VREF > 3.3f) FORCE_B_VREF = 3.3f;
                if (FORCE_B_VREF < 0.0f) FORCE_B_VREF = 0.0f;
                dac_2 = FORCE_B_VREF / 3.3f;
                // dac_2 = 1.65f / 3.3f;
            }
        }
        TMR3_COUNT_TORQUE_NULL++;
    } else {
        if (SENSING_MODE == 0) { // Force Sensor (Loadcell)
            FORCE_pulse_sum = 0.0f;
            dac_1 = FORCE_VREF / 3.3f;
            spi_eeprom_write(RID_FORCE_SENSOR_VREF, (int16_t)(FORCE_VREF * 1000.0f));
        } else if (SENSING_MODE == 1) {
            PresA_pulse_sum = 0.0f;
            PresB_pulse_sum = 0.0f;
            dac_1 = PRES_A_VREF / 3.3f;
            dac_2 = PRES_B_VREF / 3.3f;
            spi_eeprom_write(RID_PRES_A_SENSOR_VREF, (int16_t)(PRES_A_VREF * 1000.0f));
            spi_eeprom_write(RID_PRES_B_SENSOR_VREF, (int16_t)(PRES_B_VREF * 1000.0f));
        } else if (SENSING_MODE == 2) { // Force Sensor (Loadcell)
            FORCE_pulse_sum = 0.0f;
            dac_2 = FORCE_B_VREF / 3.3f;
            spi_eeprom_write(RID_FORCE_SENSOR_VREF_B, (int16_t)(FORCE_B_VREF * 1000.0f));
        } 
        CONTROL_UTILITY_MODE = MODE_NO_ACT;
        TMR3_COUNT_TORQUE_NULL = 0;
        // ALART_FLAG_ON(ALART_UTILFUNC_DONE);
    }
}

void UtilFunc_FIND_HOME(void)
{
    if (FINDHOME_STAGE == FINDHOME_INIT) {
        REFERENCE_MODE = MODE_REF_UTILMODE;
        CNT4UtilityMode = 0;
        cnt_touch_end = 0;
        pos.ref = pos.sen;
        vel.ref = 0.0f;
        UtilityMode_PosRef = pos.sen;
        UtilityMode_Pos = pos.sen;
        UtilityMode_PosOld = pos.sen;
        FINDHOME_STAGE = FINDHOME_GOTOLIMIT;
    } else if (FINDHOME_STAGE == FINDHOME_GOTOLIMIT) {

        float GOTOHOME_SPEED = 20.0f;       // 20mm/s or 20deg/s
        if (HOMEPOS_OFFSET > 0) {
            UtilityMode_PosRef = UtilityMode_PosRef + GOTOHOME_SPEED * DT_5k;
        } else {
            UtilityMode_PosRef = UtilityMode_PosRef - GOTOHOME_SPEED * DT_5k;
        }
        Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

        CNT4UtilityMode++;
        if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz checking
            UtilityMode_Pos = pos.sen;
            UtilityMode_Vel = UtilityMode_Pos - UtilityMode_PosOld;
            UtilityMode_PosOld = UtilityMode_Pos;
            CNT4UtilityMode = 0;
        }

        if(fabs(UtilityMode_Vel) <= GOTOHOME_SPEED*0.1f) {
            cnt_touch_end = cnt_touch_end + 1;
        } else {
            cnt_touch_end = 0;
        }

        if ((cnt_touch_end >= 3 * TMR_FREQ_5k) || CNT4UtilityMode >= 10 * TMR_FREQ_5k) { // wait for 3sec
            ENC_SET((long)((long)HOMEPOS_OFFSET));
            Flag_PosMidFound = false; 
            UtilityMode_PosRef_INIT = ((float)HOMEPOS_OFFSET)/ENC_PULSE_PER_POSITION; // pulse >> deg or mm
            UtilityMode_Pos = 0;
            UtilityMode_PosOld = 0;
            UtilityMode_Vel = 0;

            CNT4UtilityMode = 0;
            cnt_touch_end = 0;
            pos.ref = 0.0f;
            FINDHOME_STAGE = FINDHOME_ZEROPOSE;
        }
    } else if (FINDHOME_STAGE == FINDHOME_ZEROPOSE) {
        int T_move = 2*TMR_FREQ_5k;
        UtilityMode_PosRef = UtilityMode_PosRef_INIT + (0.0f - UtilityMode_PosRef_INIT) * 0.5f *(1.0f - cosf(PI * (float)CNT4UtilityMode / (float)T_move));
        Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

        CNT4UtilityMode++;
        if (CNT4UtilityMode >= T_move) {
            CNT4UtilityMode = 0;
            pos.ref = 0.0f;
            vel.ref = 0.0f;
            Flag_FINDHOME_DONE = true;
            // CONTROL_UTILITY_MODE = MODE_NO_ACT;
            CONTROL_UTILITY_MODE = MODE_JOINT_CONTROL;
            REFERENCE_MODE = MODE_REF_EXTERNAL;
            FINDHOME_STAGE = FINDHOME_INIT;
            ALART_FLAG_ON(ALART_UTILFUNC_DONE);
        }
    }
}

void UtilFunc_ID_VALVEPOS_VS_PWM(void) 
{
    CONTROL_MODE = MODE_VALVE_OPEN_LOOP;

    switch(STAGE_VALVEPOS_VS_PWM_ID) {
        case VALVEPOS_VS_PWM_ID_INIT:
        {
            MODE_LED_Display = 1;
            Vout.ref = 0.0f;
            UtilityMode_ID_index = 0;
            STAGE_VALVEPOS_VS_PWM_ID++;
            break;
        } 
        case VALVEPOS_VS_PWM_ID_MAIN:
        {
            CNT4UtilityMode = CNT4UtilityMode + 1;
            if (CNT4UtilityMode < TMR_FREQ_5k * 1) {
                Vout.ref = 0.0f;
            } else if (CNT4UtilityMode < TMR_FREQ_5k * 2) {
                Vout.ref = 1000.0f * PWMs_for_ValvePosID[UtilityMode_ID_index];
            } else if (CNT4UtilityMode == TMR_FREQ_5k * 2) {
                VALVE_POS_SUM = 0.0f;
                data_num = 0;
            } else if (CNT4UtilityMode < TMR_FREQ_5k * 3) {
                data_num = data_num + 1;
                VALVE_POS_SUM = VALVE_POS_SUM + valve_pos.sen;
            } else if (CNT4UtilityMode == TMR_FREQ_5k * 3) {
                Vout.ref = 0.0f;
                VALVE_POS_AVG[UtilityMode_ID_index] = VALVE_POS_SUM / (float)data_num; // Unit : pulse (0~4095)
                CNT4UtilityMode = 0;
                UtilityMode_ID_index = UtilityMode_ID_index + 1;
            } 
            
            if (UtilityMode_ID_index == NUM_VALVEPOS_VS_PWM_ID) {                
                STAGE_VALVEPOS_VS_PWM_ID++;
            }
            break;
        }
        case VALVEPOS_VS_PWM_ID_TERMINATE:
        {
            MODE_LED_Display = 3;
            VALVE_MAX_POS = (int)VALVE_POS_AVG[0];
            VALVE_MIN_POS = (int)VALVE_POS_AVG[0];
            VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index-1)/2] = VALVE_POS_AVG[0];       
            if(VALVE_MAX_POS+5.0f<VALVE_POS_AVG[1]) {
                VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index+1)/2] = VALVE_POS_AVG[1];   
                VALVE_MAX_POS = VALVE_POS_AVG[1];   
            } else {
                VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index+1)/2] = VALVE_MAX_POS + 5.0f;   
                VALVE_MAX_POS = VALVE_MAX_POS + 5.0f;   
            }    
            if(VALVE_MIN_POS-5.0f>VALVE_POS_AVG[2]) {
                VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index-3)/2] = VALVE_POS_AVG[2];                   
                VALVE_MIN_POS = VALVE_POS_AVG[2];   
            } else {
                VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index-3)/2] = VALVE_MIN_POS - 5.0f;   
                VALVE_MIN_POS = VALVE_MIN_POS - 5.0f;   
            }    

            for (int i = 3; i < UtilityMode_ID_index; i++) {
                if(i % 2 == 0) { // (-) direction PWM
                    if(VALVE_MIN_POS-5.0f>VALVE_POS_AVG[i]) {
                        VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index-i-1)/2] = VALVE_POS_AVG[i];   
                        VALVE_MIN_POS = VALVE_POS_AVG[i];
                    } else {
                        VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index-i-1)/2] = VALVE_MIN_POS - 5.0f;   
                        VALVE_MIN_POS = VALVE_MIN_POS - 5.0f;   
                    }         
                } else {
                    if(VALVE_MAX_POS+5.0f<VALVE_POS_AVG[i]) {
                        VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index+i)/2] = VALVE_POS_AVG[i];   
                        VALVE_MAX_POS = VALVE_POS_AVG[i];   
                    } else {
                        VALVE_POS_VS_PWM_Sorted[(UtilityMode_ID_index+i)/2] = VALVE_MAX_POS + 5.0f;   
                        VALVE_MAX_POS = VALVE_MAX_POS + 5.0f;   
                    }    
                }
            }

            VALVE_ELECTRIC_CENTER = (int)VALVE_POS_VS_PWM[0];
            // spi_eeprom_write(RID_VALVE_ELECTRIC_CENTER, (int16_t)VALVE_ELECTRIC_CENTER);
            // spi_eeprom_write(RID_VALVE_MAX_POS, (int16_t)VALVE_MAX_POS);
            // spi_eeprom_write(RID_VALVE_MIN_POS, (int16_t)VALVE_MIN_POS);
            for (int i = 0; i < NUM_VALVEPOS_VS_PWM_ID; i++) {
                spi_eeprom_write(RID_VALVE_POS_VS_PWM_0 + i, (int16_t)VALVE_POS_VS_PWM_Sorted[i]);
            }
            UtilityMode_ID_index = 0;
            CNT4UtilityMode = 0;
            CONTROL_UTILITY_MODE = MODE_NO_ACT;
            STAGE_VALVEPOS_VS_PWM_ID = VALVEPOS_VS_PWM_ID_INIT;
            // ALART_FLAG_ON(ALART_UTILFUNC_DONE);
            break;
        }
        default: 
            break;
    }
}

void UtilFunc_ID_DEADZONE_AND_CENTER(void) 
{
    switch(STAGE_VALVEDZ_ID) {
        case VALVEDZ_ID_INIT: 
        {
            Vout.ref = 0.0f;
            UtilityMode_PosRef = pos.sen;
            UtilityMode_Pos = pos.sen;
            UtilityMode_PosOld = pos.sen;
            cnt_touch_end = 0;
            CNT4UtilityMode = 0;
            STAGE_VALVEDZ_ID = VALVEDZ_ID_FIND_POS_PLUS;
            break;
        }
        case VALVEDZ_ID_FIND_POS_PLUS: 
        {
            float GOTOEND_SPEED = 20.0f; // 20mm/s or 20deg/s
            UtilityMode_PosRef = UtilityMode_PosRef + GOTOEND_SPEED * DT_5k;
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            
            CNT4UtilityMode = CNT4UtilityMode + 1;
            if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz Checking
                UtilityMode_Pos = pos.sen;
                UtilityMode_Vel = (UtilityMode_Pos - UtilityMode_PosOld)*10.0f; // 10Hz Checking
                UtilityMode_PosOld = UtilityMode_Pos;
                CNT4UtilityMode = 0;
            }

            if(fabs(UtilityMode_Vel) <= GOTOEND_SPEED*0.1f) {
                cnt_touch_end = cnt_touch_end + 1;
            } else {
                cnt_touch_end = 0;
            }

            if(cnt_touch_end > 2 * TMR_FREQ_5k)  { 
                pos_plus_end = (int)pos.sen;  
                UtilityMode_PosRef = pos.sen;
                UtilityMode_Pos = pos.sen;
                UtilityMode_PosOld = pos.sen;
                cnt_touch_end = 0;
                CNT4UtilityMode = 0;
                STAGE_VALVEDZ_ID = VALVEDZ_ID_FIND_POS_MINUS;
            } 
            break;
        }
        case VALVEDZ_ID_FIND_POS_MINUS:
        {
            float GOTOEND_SPEED = 20.0f; // 20mm/s or 20deg/s
            UtilityMode_PosRef = UtilityMode_PosRef - GOTOEND_SPEED * DT_5k;
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            
            CNT4UtilityMode = CNT4UtilityMode + 1;
            if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz Checking
                UtilityMode_Pos = pos.sen;
                UtilityMode_Vel = (UtilityMode_Pos - UtilityMode_PosOld)*10.0f; // 10Hz Checking
                UtilityMode_PosOld = UtilityMode_Pos;
                CNT4UtilityMode = 0;
            }

            if(fabs(UtilityMode_Vel) <= GOTOEND_SPEED*0.1f) {
                cnt_touch_end = cnt_touch_end + 1;
            } else {
                cnt_touch_end = 0;
            }

            if(cnt_touch_end > 2 * TMR_FREQ_5k)  { 
                pos_minus_end = (int)pos.sen;  
                pos_mid = (pos_plus_end+pos_minus_end)/2; 
                Flag_PosMidFound = true; 
                UtilityMode_PosRef = pos.sen;
                UtilityMode_Pos = pos.sen;
                UtilityMode_PosOld = pos.sen;
                cnt_touch_end = 0;
                CNT4UtilityMode = 0;
                STAGE_VALVEDZ_ID = VALVEDZ_ID_MOVE2MID;  
            } 
            break;
        }
        case VALVEDZ_ID_MOVE2MID: 
        {
            int GOTOMID_TIME = 3; // 3sec
            UtilityMode_PosRef = (float)pos_minus_end + 
                                        0.5f*(1.0f-cosf(PI*(float)CNT4UtilityMode*DT_5k/(float)GOTOMID_TIME))*((float)pos_mid-(float)pos_minus_end);
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            CNT4UtilityMode++;
            if(CNT4UtilityMode>TMR_FREQ_5k*GOTOMID_TIME) {
                CNT4UtilityMode = 0;
                VALVE_POS_AVG[0] = 0.0f;
                data_num = 0;
                STAGE_VALVEDZ_ID = VALVEDZ_ID_FIND_STARTPOINT;  
            }
            break;
        } 
        case VALVEDZ_ID_FIND_STARTPOINT:
        {
            CNT4UtilityMode++;
            if (CNT4UtilityMode<=TMR_FREQ_5k*2) { // Sampling Time : 2 sec >> 10000 samples
                Vout.ref = PosCtrl4UtilFunc((float)pos_mid);
                VALVE_POS_AVG[0] = VALVE_POS_AVG[0] + valve_pos.sen/(FREQ_5k*2.0f);
                if (CNT4UtilityMode==TMR_FREQ_5k*2) {
                    CNT4UtilityMode = 0;
                    START_POS = pos.sen;
                    Ref_Valve_Pos_Old = VALVE_POS_AVG[0];
                    UtilityMode_ID_index = 1;
                    DZ_DIRECTION = 1;
                    STAGE_VALVEDZ_ID = VALVEDZ_ID_FIND_DZBAND_LOWERBOUND;  
                }
            } 
            break;
        } 
        case VALVEDZ_ID_FIND_DZBAND_LOWERBOUND:
        {
            CNT4UtilityMode++;
            if (CNT4UtilityMode <= TMR_FREQ_5k*1) { 
                // Ref_Valve_Pos_Old 입력 시 (+) 방향으로 치우치는 있는 경우에는 (-) 방향으로,
                // (-) 방향으로 치우치는 있는 경우에는 (+) 방향으로 offset을 줌.
                valve_pos_raw.ref = Ref_Valve_Pos_Old - (float)DZ_DIRECTION*DZ_OPENSIZE/(float)UtilityMode_ID_index; // valve_pos_raw.ref는 0~4095 사이. 적절하게 열어주면서 Encoder 위치 check                            
                if (valve_pos_raw.ref <= VALVE_MIN_POS) {
                    valve_pos_raw.ref = VALVE_MIN_POS;
                } else if (valve_pos_raw.ref >= VALVE_MAX_POS) {
                    valve_pos_raw.ref = VALVE_MAX_POS;
                }

                Vout.ref = VALVE_POS_CONTROL(valve_pos_raw.ref);

                if (CNT4UtilityMode == TMR_FREQ_5k*1) {
                    FINAL_POS = pos.sen;
                    if ((FINAL_POS - START_POS)*ENC_PULSE_PER_POSITION > 5.0f) { // If the rod displacement is over 5 pulse, the opening isn't in DZ.
                        DZ_DIRECTION = 1;
                    } else if ((FINAL_POS - START_POS)*ENC_PULSE_PER_POSITION < -5.0f) {
                        DZ_DIRECTION = -1;
                    } else {
                        DZ_DIRECTION = 1;
                    }
                    Ref_Valve_Pos_Old = valve_pos_raw.ref;
                }
            } else if (CNT4UtilityMode <= TMR_FREQ_5k*4) {
                int GOTOMID_TIME = 3; // 3sec
                UtilityMode_PosRef = FINAL_POS + 
                                            0.5f*(1.0f-cosf(PI*((float)CNT4UtilityMode-(float)TMR_FREQ_5k)*DT_5k/(float)GOTOMID_TIME))*((float)pos_mid-FINAL_POS);
                Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

                if (CNT4UtilityMode == TMR_FREQ_5k*4) {
                    START_POS = pos.sen;
                    CNT4UtilityMode = 0;
                    if (UtilityMode_ID_index >= 128) {
                        FIRST_DZ = valve_pos_raw.ref;
                        Ref_Valve_Pos_Old = FIRST_DZ;
                        START_POS = pos.sen;
                        UtilityMode_ID_index = 1;
                        DZ_DIRECTION = -1;
                        STAGE_VALVEDZ_ID = VALVEDZ_ID_FIND_DZBAND_UPPERBOUND;
                    } else {
                        UtilityMode_ID_index = UtilityMode_ID_index * 2;
                    }
                }    
            } 
            break;
        }
        case VALVEDZ_ID_FIND_DZBAND_UPPERBOUND:
            {
            CNT4UtilityMode++;
            if (CNT4UtilityMode <= TMR_FREQ_5k*1) { 
                // Ref_Valve_Pos_Old 입력 시 (+) 방향으로 치우치는 있는 경우에는 (-) 방향으로,
                // (-) 방향으로 치우치는 있는 경우에는 (+) 방향으로 offset을 줌.
                valve_pos_raw.ref = Ref_Valve_Pos_Old - (float)DZ_DIRECTION*DZ_OPENSIZE/(float)UtilityMode_ID_index; // valve_pos_raw.ref는 0~4095 사이. 적절하게 열어주면서 Encoder 위치 check                            
                if (valve_pos_raw.ref <= VALVE_MIN_POS) {
                    valve_pos_raw.ref = VALVE_MIN_POS;
                } else if (valve_pos_raw.ref >= VALVE_MAX_POS) {
                    valve_pos_raw.ref = VALVE_MAX_POS;
                }

                Vout.ref = VALVE_POS_CONTROL(valve_pos_raw.ref);

                if (CNT4UtilityMode == TMR_FREQ_5k*1) {
                    FINAL_POS = pos.sen;
                    if ((FINAL_POS - START_POS)*ENC_PULSE_PER_POSITION > 5.0f) { // If the rod displacement is over 5 pulse, the opening isn't in DZ.
                        DZ_DIRECTION = 1;
                    } else if ((FINAL_POS - START_POS)*ENC_PULSE_PER_POSITION < -5.0f) {
                        DZ_DIRECTION = -1;
                    } else {
                        DZ_DIRECTION = -1;
                    }
                    Ref_Valve_Pos_Old = valve_pos_raw.ref;
                }
            } else if (CNT4UtilityMode <= TMR_FREQ_5k*4) {
                int GOTOMID_TIME = 3; // 3sec
                UtilityMode_PosRef = FINAL_POS + 
                                            0.5f*(1.0f-cosf(PI*((float)CNT4UtilityMode-(float)TMR_FREQ_5k)*DT_5k/(float)GOTOMID_TIME))*((float)pos_mid-FINAL_POS);
                Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

                if (CNT4UtilityMode == TMR_FREQ_5k*4) {
                    START_POS = pos.sen;
                    CNT4UtilityMode = 0;
                    if (UtilityMode_ID_index >= 128) {
                        SECOND_DZ = valve_pos_raw.ref;
                        UtilityMode_ID_index = 1;
                        DZ_DIRECTION = 1;
                        VALVE_CENTER = (int16_t)(FIRST_DZ+SECOND_DZ)/2;
                        VALVE_DEADZONE_MINUS = (int16_t)FIRST_DZ;
                        VALVE_DEADZONE_PLUS = (int16_t)SECOND_DZ;
                        spi_eeprom_write(RID_VALVE_CENTER, (int16_t)VALVE_CENTER);
                        spi_eeprom_write(RID_VALVE_DEADZONE_PLUS, (int16_t)VALVE_DEADZONE_PLUS);
                        spi_eeprom_write(RID_VALVE_DEADZONE_MINUS, (int16_t)VALVE_DEADZONE_MINUS);
                        STAGE_VALVEDZ_ID = VALVEDZ_ID_TERMINATE;
                    } else {
                        UtilityMode_ID_index = UtilityMode_ID_index * 2;
                    }                         
                }    
            } 
            break;
        }
        case VALVEDZ_ID_TERMINATE:
        {
            int GOTOHOME_TIME = 3; // 3sec
            UtilityMode_PosRef = (float)START_POS + 
                                        0.5f*(1.0f-cosf(PI*(float)CNT4UtilityMode*DT_5k/(float)GOTOHOME_TIME))*((float)pos_minus_end-(float)START_POS);
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            CNT4UtilityMode++;
            if(CNT4UtilityMode>TMR_FREQ_5k*GOTOHOME_TIME) {
                // CONTROL_UTILITY_MODE = MODE_JOINT_CONTROL;
                // REFERENCE_MODE = MODE_REF_DIRECT;
                CONTROL_UTILITY_MODE = MODE_NO_ACT;
                CNT4UtilityMode = 0;
                STAGE_VALVEDZ_ID = VALVEDZ_ID_INIT;  
                // ALART_FLAG_ON(ALART_UTILFUNC_DONE);
            }            
            break;
        }
        default: 
            break;
    }
}

void UtilFunc_ID_FLOWRATE_VS_VALVEPOS(void)
{
    switch(STAGE_FLOWRATE_VS_VALVEPOS_ID) {
        case FLOWRATE_VS_VALVEPOS_ID_INIT:
        {
            Vout.ref = 0.0f;
            UtilityMode_PosRef = pos.sen;
            UtilityMode_Pos = pos.sen;
            UtilityMode_PosOld = pos.sen;
            cnt_touch_end = 0;
            CNT4UtilityMode = 0;
            STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_FIND_POS_PLUS;
            break;
        }
        case FLOWRATE_VS_VALVEPOS_ID_FIND_POS_PLUS: 
        {
            float GOTOEND_SPEED = 20.0f; // 20mm/s or 20deg/s
            UtilityMode_PosRef = UtilityMode_PosRef + GOTOEND_SPEED * DT_5k;
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            
            CNT4UtilityMode = CNT4UtilityMode + 1;
            if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz Checking
                UtilityMode_Pos = pos.sen;
                UtilityMode_Vel = (UtilityMode_Pos - UtilityMode_PosOld)*10.0f; // 10Hz Checking
                UtilityMode_PosOld = UtilityMode_Pos;
                CNT4UtilityMode = 0;
            }

            if(fabs(UtilityMode_Vel) <= GOTOEND_SPEED*0.1f) {
                cnt_touch_end = cnt_touch_end + 1;
            } else {
                cnt_touch_end = 0;
            }

            if(cnt_touch_end > 2 * TMR_FREQ_5k)  { 
                pos_plus_end = (int)pos.sen;  
                UtilityMode_PosRef = pos.sen;
                UtilityMode_Pos = pos.sen;
                UtilityMode_PosOld = pos.sen;
                cnt_touch_end = 0;
                CNT4UtilityMode = 0;
                STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_FIND_POS_MINUS;
            } 
            break;
        }
        case FLOWRATE_VS_VALVEPOS_ID_FIND_POS_MINUS: 
        {
            float GOTOEND_SPEED = 20.0f; // 20mm/s or 20deg/s
            UtilityMode_PosRef = UtilityMode_PosRef - GOTOEND_SPEED * DT_5k;
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);
            
            CNT4UtilityMode = CNT4UtilityMode + 1;
            if (CNT4UtilityMode >= (TMR_FREQ_5k / 10)) { // 10Hz Checking
                UtilityMode_Pos = pos.sen;
                UtilityMode_Vel = (UtilityMode_Pos - UtilityMode_PosOld)*10.0f; // 10Hz Checking
                UtilityMode_PosOld = UtilityMode_Pos;
                CNT4UtilityMode = 0;
            }

            if(fabs(UtilityMode_Vel) <= GOTOEND_SPEED*0.1f) {
                cnt_touch_end = cnt_touch_end + 1;
            } else {
                cnt_touch_end = 0;
            }

            if(cnt_touch_end > 2 * TMR_FREQ_5k)  { 
                pos_minus_end = (int)pos.sen;  
                pos_mid = (pos_plus_end+pos_minus_end)/2; 
                Flag_PosMidFound = true; 
                UtilityMode_PosRef = pos.sen;
                UtilityMode_Pos = pos.sen;
                UtilityMode_PosOld = pos.sen;
                cnt_touch_end = 0;
                CNT4UtilityMode = 0;
                STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_MOVE2MID;  
            } 
            break;
        }
        case FLOWRATE_VS_VALVEPOS_ID_MOVE2MID: 
        {
            int GOTOMID_TIME = 3; // 3sec
            UtilityMode_PosRef = (float)pos_minus_end + 
                                        0.5f*(1.0f-cosf(PI*(float)CNT4UtilityMode*DT_5k/(float)GOTOMID_TIME))*((float)pos_mid-(float)pos_minus_end);
            Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

            CNT4UtilityMode++;
            if(CNT4UtilityMode>TMR_FREQ_5k*GOTOMID_TIME) {
                CNT4UtilityMode = 0;
                UtilityMode_ID_index = 0;
                START_POS = pos.sen;
                STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_MAIN;  
            }
            break;
        } 
        case FLOWRATE_VS_VALVEPOS_ID_MAIN: 
        {
            CNT4UtilityMode++;

            if (CNT4UtilityMode <= TMR_FREQ_5k*1) {
                valve_pos_raw.ref = (float)VALVE_CENTER + ValvePosOff_for_FlowrateID[UtilityMode_ID_index];    
                if (valve_pos_raw.ref <= VALVE_MIN_POS) {
                    valve_pos_raw.ref = VALVE_MIN_POS;
                } else if (valve_pos_raw.ref >= VALVE_MAX_POS) {
                    valve_pos_raw.ref = VALVE_MAX_POS;
                }
                Vout.ref = VALVE_POS_CONTROL(valve_pos_raw.ref);

                if((pos.sen < (float)pos_minus_end + 10.0)||(pos.sen > (float)pos_plus_end - 10.0))  {
                    cnt_touch_end++;
                    CNT4UtilityMode_Old = CNT4UtilityMode;
                } 

                if (cnt_touch_end != 0 || CNT4UtilityMode == TMR_FREQ_5k*1) {
                    FINAL_POS = pos.sen;
                    float temp_vel = (FINAL_POS - START_POS)/(float)CNT4UtilityMode*(float)TMR_FREQ_5k; // deg/s or mm/s
                    if(Classify_ActuatorType() == ACTUATORTYPE_ROT) { // Rotary Actuator Mode
                        if(temp_vel>0.0f) {
                            FlowRate_VS_ValvePosOff[UtilityMode_ID_index] = temp_vel*PI/180.0f*(float)PISTON_AREA_A/1000.0f*60.0f/1000.0f;
                        } else {
                            FlowRate_VS_ValvePosOff[UtilityMode_ID_index] = temp_vel*PI/180.0f*(float)PISTON_AREA_B/1000.0f*60.0f/1000.0f;
                        }
                    } else { // Linear Actuator Mode
                        if(temp_vel>0.0f) {
                            FlowRate_VS_ValvePosOff[UtilityMode_ID_index] = temp_vel*(float)PISTON_AREA_A/1000.0f*60.0f/1000.0f;
                        } else {
                            FlowRate_VS_ValvePosOff[UtilityMode_ID_index] = temp_vel*(float)PISTON_AREA_B/1000.0f*60.0f/1000.0f;
                        }                            
                    }
                    STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_RETURN2MID;  
                    CNT4UtilityMode = 0;
                    CNT4UtilityMode_Old = 0;
                    cnt_touch_end = 0;
                }
            } 
            break;
        }
        case FLOWRATE_VS_VALVEPOS_ID_RETURN2MID: 
        {
            int GOTOMID_TIME = 2; // 2sec
            CNT4UtilityMode++;
            if(CNT4UtilityMode <= TMR_FREQ_5k*GOTOMID_TIME) {
                UtilityMode_PosRef = FINAL_POS + 
                                        0.5f*(1.0f-cosf(PI*(float)CNT4UtilityMode*DT_5k/(float)GOTOMID_TIME))*((float)pos_mid-FINAL_POS);
                Vout.ref = PosCtrl4UtilFunc(UtilityMode_PosRef);

                if (CNT4UtilityMode == TMR_FREQ_5k*GOTOMID_TIME) {
                    START_POS = pos.sen;
                    CNT4UtilityMode = 0;
                    UtilityMode_ID_index++;
                    if (UtilityMode_ID_index == NUM_FLOWRATE_VS_VALVEPOS_ID) {
                        STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_TERMINATE;
                    } else {
                        STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_MAIN;
                    }  
                }
            }
            break;
        }
        case FLOWRATE_VS_VALVEPOS_ID_TERMINATE:
        {
            float Qmin = 0.0f, Qmax = 0.0f;
            float FlowRate_VS_ValvePosOff_temp[NUM_FLOWRATE_VS_VALVEPOS_ID] = {0.0f,};
            Qmin = FlowRate_VS_ValvePosOff[0];
            Qmax = FlowRate_VS_ValvePosOff[0];
            FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index-1)/2] = FlowRate_VS_ValvePosOff[0];

            if(Qmax+0.001f<FlowRate_VS_ValvePosOff[1]) {
                FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index+1)/2] = FlowRate_VS_ValvePosOff[1];   
                Qmax = FlowRate_VS_ValvePosOff[1];   
            } else {
                FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index+1)/2] = Qmax + 0.001f;   
                Qmax = Qmax + 0.001f;   
            }    
            if(Qmin-0.001f>FlowRate_VS_ValvePosOff[2]) {
                FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index-3)/2] = FlowRate_VS_ValvePosOff[2];                   
                Qmin = FlowRate_VS_ValvePosOff[2];   
            } else {
                FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index-3)/2] = Qmin - 0.001f;   
                Qmin = Qmin - 0.001f;   
            }    

            for (int i = 3; i < UtilityMode_ID_index; i++) {
                if(i % 2 == 0) { // (-) direction PWM
                    if(Qmin-0.001f>FlowRate_VS_ValvePosOff[i]) {
                        FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index-i-1)/2] = FlowRate_VS_ValvePosOff[i];   
                        Qmin = FlowRate_VS_ValvePosOff[i];
                    } else {
                        FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index-i-1)/2] = Qmin - 0.001f;   
                        Qmin = Qmin - 0.001f;   
                    }         
                } else {
                    if(Qmax+0.001f<FlowRate_VS_ValvePosOff[i]) {
                        FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index+i)/2] = FlowRate_VS_ValvePosOff[i];   
                        Qmax = FlowRate_VS_ValvePosOff[i];   
                    } else {
                        FlowRate_VS_ValvePosOff_temp[(UtilityMode_ID_index+i)/2] = Qmax + 0.001f;   
                        Qmax = Qmax + 0.001f;   
                    }    
                }
            }
            
            // float mask[5] = {0.0f,0.0f,1.0f,0.0f,0.0f};
            float mask[5] = {0.5f,0.0f,0.0f,0.0f,0.5f};
            ArrayMasking_2nd(FlowRate_VS_ValvePosOff_temp, FlowRate_VS_ValvePosOff_Sorted, NUM_FLOWRATE_VS_VALVEPOS_ID, mask);

            for (int i = 0; i < UtilityMode_ID_index; i++) {
                spi_eeprom_write(RID_VALVE_POS_VS_FLOWRATE_0 + i, (int16_t)(FlowRate_VS_ValvePosOff_Sorted[i]*1000.0f));
            }
            UtilityMode_ID_index = 0;
            CNT4UtilityMode = 0;

            // CONTROL_UTILITY_MODE = MODE_JOINT_CONTROL;
            // REFERENCE_MODE = MODE_REF_DIRECT;
            CONTROL_UTILITY_MODE = MODE_NO_ACT;
            STAGE_FLOWRATE_VS_VALVEPOS_ID = FLOWRATE_VS_VALVEPOS_ID_INIT;
            // ALART_FLAG_ON(ALART_UTILFUNC_DONE);
            break;
        }
        default:
            break;
    }
}
