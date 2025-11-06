#include <Bounce.h> //input debouce library
#include <FlexCAN_T4.h>
#include <InternalTemperature.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; // Using CAN1 on pins 22/rx & 23/tx
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // Using CAN2 on pins 0/rx & 1/tx
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3; // Using CAN3 on pins 30/rx & 31/tx

//reboot
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)

//PWM
#define pwmPumpHigh_off 450
#define pwmPumpHigh_lowFlow 200
#define pwmPumpHigh_midFlow 130
#define pwmPumpHigh_hiFlow 50

#define pwmFan_off 255
#define pwmFan_low 150
#define pwmFan_mid 100
#define pwmFan_hi 50
#define pwmFan_freq 120

//bms status
#define BMS_Boot 0
#define BMS_Ready 1
#define BMS_Drive 2
#define BMS_Charge 3
#define BMS_Precharge 4
#define BMS_Error 5

//drive status
#define Drive_N 0
#define Drive_F 1
#define Drive_R -1
#define Drive_Invalid 15

#define Drive_OPMODE_Off 0
#define Drive_OPMODE_Run 1
#define Drive_OPMODE_ManualRun 2
#define Drive_OPMODE_Boost 3
#define Drive_OPMODE_Buck 4
#define Drive_OPMODE_Sine 5
#define Drive_OPMODE_ACHeat 6
#define Drive_OPMODE_Invalid 15

#define Drive_Stat_None 0
#define Drive_Stat_VdcLow 1
#define Drive_Stat_VdcHigh 2
#define Drive_Stat_VdcBelowUdcSw 4
#define Drive_Stat_VdcLim 8
#define Drive_Stat_EmcyStop 16
#define Drive_Stat_MProt 32
#define Drive_Stat_PotPressed 64
#define Drive_Stat_TmpHs 128
#define Drive_Stat_WaitStart 256
#define Drive_Stat_BrakeCheck 512
#define Drive_Stat_Invalid 0xFFFF


//charge status
#define Charge_NotReady 0
#define Charge_Initialisation 1
#define Charge_Active 2
#define Charge_Pause 3
#define Charge_Completed 4
#define Charge_Error 5
#define Charge_Invalid 6

//charge request constants
#define ChargeReq_EndCharge 0
#define ChargeReq_Charge 1

//charge ready constants
#define ChargeReady_No 0
#define ChargeReady_Yes 1

//charge end constants
#define ChargeEnd_No 0
#define ChargeEnd_Yes 1
#define ChargeEnd_Invalid 3

//charge ready constants - DC
#define DCChargeEnd_No 0
#define DCChargeEnd_Yes 1
#define DCChargeEnd_Invalid 3

//dc charge phases
#define DCChargePhase_Standby 0
#define DCChargePhase_Initialisation 1
#define DCChargePhase_Subpoena 2
#define DCChargePhase_EnergyTransfer 3
#define DCChargePhase_Shutdown 4
#define DCChargePhase_CableTest 9
#define DCChargePhase_Reserved 14
#define DCChargePhase_InvalidSignal 15

//ibooster messages
//IBST_driverBrakeApply
#define BRAKES_NOT_APPLIED 1
#define DRIVER_APPLYING_BRAKES 2
#define FAULT 3
#define NOT_INIT_OR_OFF 0
//IBST_iBoosterStatus
#define IBOOSTER_ACTIVE_GOOD_CHECK 4
#define IBOOSTER_ACTUATION 6
#define IBOOSTER_DIAGNOSTIC 3
#define IBOOSTER_FAILURE 2
#define IBOOSTER_INIT 1
#define IBOOSTER_OFF 0
#define IBOOSTER_READY 5
//IBST_internalState
#define DIAGNOSTIC 4
#define EXTERNAL_BRAKE_REQUEST 3
#define LOCAL_BRAKE_REQUEST 2
#define NO_MODE_ACTIVE 0
#define POST_DRIVE_CHECK 6
#define PRE_DRIVE_CHECK 1
#define TRANSITION_TO_IDLE 5
#define IBST_Invalid 0xF

// pin assignments
#define pwmOutPumpMain 3
#define pwmOutPumpBat 2
#define dout4Spare 4 
#define dout5Spare 5 
#define dout6Spare 6 
#define dout7Spare 7
#define outBMSChargeReq 8
#define outDCDCenable 9
#define outOBCenable 10
#define dout11Spare 11
#define flapSimulatorOut 12
#define pwmOutFanMain 24
#define pwmOutFanBat 25
#define inStopCharge 17
#define inIgnition 18
#define din19Spare 19
#define din20Spare 20
#define din21Spare 21

//CPU
float VCUtempC=0;

//watchdog
uint32_t Debug_Stamp = millis();
uint32_t IBooster_watchdog = millis();
uint32_t LIM_watchdog = millis();
uint32_t ISA_watchdog = millis();
uint32_t BMS_watchdog = millis();
uint32_t Elcon_watchdog = millis();
uint32_t Drive_watchdog = millis();
uint32_t Timeout = millis();
uint32_t LED_watchdog = millis();
uint32_t PWM_MPump_Stamp = millis();
uint32_t PWM_BPump_Stamp = millis();
uint32_t CoolingFanPumpDelay_Stamp = millis();// fan delayed start
uint32_t DCDCPre_Stamp = millis(); //delay start for DCDC after HV has been precharged
uint32_t IgnitionDelayOff_Stamp=millis(); //delay for Ignition switch off
uint32_t IgnitionDelayOn_Stamp=millis(); //delay for Ignition switch on
uint32_t Drive_Cmd_Start_Watchdog=millis(); //delay for Ignition switch off
uint32_t millis_ctr_LIM_SM = millis();  //LIM State Machine timer ensures timing of each step
uint32_t millis_ctr_LIM_SOC = millis();  //LIM SOC counter



// CAN bus setup
CAN_message_t msg;

//ibooster
const uint16_t canID_stsIBooster = 0x39D;
uint8_t IBST_driverBrakeApply=IBST_Invalid;
uint8_t IBST_iBoosterStatus=IBST_Invalid;
uint8_t IBST_internalState=IBST_Invalid;
uint16_t IBST_rodLen=0xffff;

//Drive
const uint16_t canID_Drive_Stat = 0x100; 
const uint16_t canID_Drive_Cmd = 0x3F; 
int8_t Drive_Dir=Drive_Invalid;
uint8_t Drive_OpMode=Drive_OPMODE_Invalid;
uint16_t Drive_Status=Drive_Stat_Invalid;
uint16_t Drive_MotorTemp=0;
uint16_t Drive_HtSnkTemp=0;
bool Drive_Cmd_StartPerm=false;
bool Drive_Cmd_Start=false;
bool Drive_Cmd_Cruise=false;
bool Drive_Cmd_Brake=false;
bool Drive_Cmd_Fwd=false;
bool Drive_Cmd_Rev=false;
bool Drive_Cmd_BMS=false;
uint16_t Drive_Cmd_Pot1=0;
uint16_t Drive_Cmd_Pot2=0;
uint16_t Drive_Cmd_CruiseSpeed=0;
uint16_t Drive_Cmd_RegenPreset=0;
uint8_t Drive_Cmd_Ctr=0;
uint8_t Drive_Cmd_CRC=0;

// ISA
const uint16_t  canID_ISAsoc = 0x350; // set this to match your SoC CAN bus ID
const uint16_t  canID_ISAmilliAmps = 0x521; // ISA miliAmps
const uint16_t  canID_ISAVolt1 = 0x522; // ISA voltage1
const uint16_t  canID_ISAVolt2 = 0x523; // ISA voltage1
const uint16_t  canID_ISAVolt3 = 0x525; // ISA voltage2
uint16_t ISA_BatVolt=0;                          // Battery voltage 0.1V steps up to 819V
uint16_t ISA_MainPOSVolt=0;                      // Main POS voltage battery 0.1V steps up to 818V
uint16_t ISA_MainNEGVolt=0;                      // Main POS voltage battery 0.1V steps up to 818V
uint16_t ISA_BatAmp=0;                          // Battery current 0.1A steps -819.2A to 819A
// uint16_t SOC_Bat_Act=0;                        // Battery SOC actual from ISA


//Elcon 
// Charger
const uint32_t  canID_ElconChargerCtrl = 0x1806E5F4;
const uint32_t  canID_ElconChargerFback = 0x18FF50E5;
long ElconCharger_VoltOutput=0;
long ElconCharger_AmpOutput=0;
long ElconCharger_Temp=0;
uint8_t ElconCharger_HardwareError=0xff;
uint8_t ElconCharger_TempError=0xff;
uint8_t ElconCharger_InVoltError=0xff;
uint8_t ElconCharger_BatVoltError=0xff;
uint8_t ElconCharger_CommTimeout=0xff;
bool ElconCharger_msgInvalid=false;
bool ElconCharger_CtrlReq=false;
uint16_t ElconCharger_AmpReq=0x00;
const uint16_t ElconCharger_CurrentReqMax=200; //20A DC; 0.1A scale
const uint16_t ElconCharger_CurrentReqMin=50; //5A DC; 0.1A scale
// const uint16_t ElconCharger_CurrentConsumeMax=320; //32A AC;  0.1A scale
const float ElconCharger_CurrentConst=1.6; // 32/20=1.6
// DCDC
const uint32_t  canID_ElconDCDCFback = 0x1801D08F;
long ElconDCDC_VoltOutput=0; //0.1V/bit Offset:0
long ElconDCDC_AmpOutput=0; //0.1V/bit Offset:0
long ElconDCDC_Temp=0;
uint8_t ElconDCDC_HVILError=0xff; //0:Lock accomplish,1Non-Lock
uint8_t ElconDCDC_WaterFanSts=0xff; //0:FAN OFF。1: FAN ON
uint8_t ElconDCDC_StopError=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_WaterFanSts2=0xff; //0:off。1:ON
uint8_t ElconDCDC_CommTimeout=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_HardwareError=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_Sts=0xff; //1:working, 0:stopped
uint8_t ElconDCDC_Ready=0xff; //initialisation 0:uncompleted; 1:completed
uint8_t ElconDCDC_OutOverAmp=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_InUnderVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_InOverVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OutUnderVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OutOverVolt=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_OverTemp=0xff; //1:Error, 0:No error
uint8_t ElconDCDC_HighTemp=0xff; ////1:High temp, 0:Normal temp
bool ElconDCDC_msgInvalid=false;
bool DCDC_error=false; //DCDC_error exists when started false = no error; true = error


//BMS
const uint16_t  canID_BMSLimits = 0x351; //LSB+MSB: 0,1-Charge Volt, 0.1V; 2,3 - Charge Cur, 0.1A; 4,5 Discharge Cur, 0.1A; Discharge Volt, 0.1A
const uint16_t  canID_BMSSOC = 0x355; //LSB+MSB: 0,1-SOC, 1%; 2,3-SOH,1%; 4,5-SOC, 0.01%
const uint16_t  canID_BMSStatus = 0x356; //LSB+MSB: 0,1-Volt, 0.01V; 2,3-Cur, 0.1A; 4,5-Temp, 0.1C
const uint16_t  canID_BMSAlarmsWarnings = 0x35A; //3-undervolt; 4-overvolt; 7-overtemp; 12-undertemp
const uint16_t  canID_BMSLowHigh = 0x373; //LSB+MSB: 0,1 - Min Cell Volt, 1V; 2,3 - Max Cell Volt, 1V; 4,5 - Min Cell Temp, 1K; 6,7 - Max Cell Temp, 1K; 
const uint16_t  canID_BMSInfo = 0x379; //4 - BMS Status: .0:Boot, .1:Ready, .2:Drive, .3:Charge, .4:Precharge, .5:Error
int BMS_State = 0; //
uint16_t BMS_ChargeVoltLim=0; //scale 0.1
uint16_t BMS_ChargeAmpLim=0; //scale 0.1
uint16_t BMS_disChargeVoltLim=0; //scale 0.1
uint16_t BMS_disChargeAmpLim=0; //scale 0.1
uint16_t BMS_SOC=0;
float BMS_CellsVoltMin=0;
float BMS_CellsVoltMax=0;
uint16_t BMS_CellsTempMin=0;
uint16_t BMS_CellsTempMax=0;
uint16_t BMS_CapacityAh=0;
uint32_t BMS_CapacityWhCharge=0;
uint16_t BMS_PackVolt=0;
int16_t BMS_AvgTemp=0;
long BMS_BatAmp=0;
int BMS_CellsBal=0xff;
uint8_t BMS_Error_CellOverVLimit = 0xff; //alarm: cell over high Voltage setpoint
uint8_t BMS_Error_CellUnderVLimit = 0xff; //alarm: cell under low Voltage setpoint
uint8_t BMS_Error_CellOverTLimit = 0xff; //alarm: cell over high Temperature setpoint
uint8_t BMS_Error_CellUnderTLimit = 0xff; //alarm: cell under low Temperature setpoint
uint8_t BMS_Error_CellImbalance = 0xff; //alarm: cell voltage imbalance
uint8_t BMS_Warn_CellOverVLimit = 0xff; //warning: cell over high Voltage setpoint
uint8_t BMS_Warn_CellUnderVLimit = 0xff; //warning: cell under low Voltage setpoint
uint8_t BMS_Warn_CellOverTLimit = 0xff; //warning: cell over high Temperature setpoint
uint8_t BMS_Warn_CellUnderTLimit = 0xff; //warning: cell under low Temperature setpoint

//LIM
uint8_t LIM_Stat_Hook_Pos;                     // E 337h Catch hook position, 0=no, 1=yes, 3=invalid
uint8_t LIM_Stat_Hook_Loc;                     // E 337h Catch hook locked 0=no, 1=yes, 3=invalid
uint8_t LIM_Stat_Hospitality;                  // E 390h Hospitality, 0=off, 1=on, F=invalid
uint8_t LIM_DCSE_Prot_Num=0xFF;                // E 29Eh Protocol number
uint16_t LIM_DCSE_V_Trhd=0;                    // E 29Eh Threshold voltage up to 600V in 0.1V steps
uint16_t LIM_DCSE_I_Avbl=0;                    // E 29Eh Available current up to 255A in 0.1A steps
uint16_t LIM_DCSE_V_Avbl=0;                    // E 29Eh Available voltage up to 600V in 0.1A steps
uint8_t LIM_DCSE_Stat_Iso=0b11;                // E 29Eh Isolation 0=invalid, 1=valid, 2=feler, 3=signal-invalid
uint8_t LIM_DCSE_Stat_Chg=0xF;                 // E 29Eh Internal EVSE status 0=not ready, 1=ready, 2=switch off, 3=interrupt, 4=precharge, 5=iso-monitoring, 6=emergency stop, 7=error, F=signal-invalid
uint8_t LIM_DCSE_Weld_Det=0b11;                // E 29Eh Weld detection available? 0=no, 1=yes, 3= signal invalid
uint16_t LIM_DCSE_Rst_Tme_Chg;                 // E 2B2h Remaining charging time of DC EVSE in seconds
uint8_t LIM_DCSE_V_LIM_Reached;                   // E 2B2h Voltage limit reached? 0=no 1=yes 3=invalid
uint8_t LIM_DCSE_I_LIM_Reached;                   // E 2B2h Current limit reached? 0=no 1=yes 3=invalid
uint8_t LIM_DCSE_DC_Stp_Crt;                   // E 2B2h DC charging stop 0=tracking mode, 1=suppression mode, 3=invalid
uint8_t LIM_DCSE_Bat_Prob;                     // E 2B2h Battery failure 0=normal, 1=fail. 3=invalid
uint8_t LIM_DCSE_Bat_Comp;                     // E 2B2h Battery compatible? 0=compatible, 1=incompatible
uint8_t LIM_DCSE_Loc_Stat;                     // E 2B2h Locked? 0=no, 1=yes 3=invalid
uint8_t LIM_DCSE_Chg_Prob;                     // E 2B2h charging error 0=normal, 1=field. 3=invalid
uint8_t LIM_DCSE_Chg_Stat;                     // E 2B2h Charging status 0=standby, 1=charging 3=invalid
uint16_t LIM_DCSE_I_Current;                       // E 2B2h Actual current from DC EVSE up to 255A in 0.1A st
uint16_t LIM_DCSE_V_Current;                       // E 2B2h Actual voltage from DC EVSE to 600V in 0.1V steps
uint8_t LIM_DCSE_Eng_Trsm;                     // E 2EFh Transferred kWh from the DC EVSE in 0.25kWh steps
uint8_t LIM_DCSE_Location;                     // E 2EFh 0=private, 1=office, 2=public, 3=invalid
uint8_t LIM_DCSE_P_LIM_Reached;                   // E 2EFh Power limit reached? 0=no 1=yes 3=invalid
uint16_t LIM_DCSE_I_Tol;                       // E 2EFh Current tolerance up to 20A in 0.01A steps
uint16_t LIM_DCSE_I_Riple;                     // E 2EFh Current ripple up to 20A in 0.01A steps
uint16_t LIM_DCSE_I_Min_Avbl;                  // E 2EFh Minimum available current up to 409.3A in 0.1A steps
uint16_t LIM_DCSE_V_Min_Avbl;                  // E 2EFh Minimum available voltage up to 600V in 0.1A steps
uint16_t LIM_V_CP_DC;                           // E 3B4h Voltage on the DC charging plug up to 500V in 2V steps
uint8_t LIM_Charger_Type;                       // E 3B4h Current charging type 0=no charging, 1=AC-Typ1, 2=AC-Typ2, 3=DC-Cahdemo, 4=DC-Typ1, 5=AC-CN, 6=AC-Combo1, 7=AC-Combo2, 8= DC-Typ2 9=DC-Combo2, A=DC-GB_T
uint8_t LIM_Stat_Line_Held;                    // E 3B4h Cable plugged in? 0=no, 1=completely plugged in, 2=error, 3=invalid
uint8_t LIM_Stat_Line_Chademo;                 // E 3B4h Chademo cable plugged in? 0=no, 1=yes, 2=error, 3=invalid
uint8_t LIM_Stat_Pilot;                        // E 3B4h Status Pilot 0=none, 1=10-95%_not_ChargeReady, 2=10-95%_ChargeReady, 3=error, 4=5%_not_ChargeReady., 5=5%_ChargeReady., 6=pilot static, 7= invalid
uint8_t LIM_Stat_Crt_Chademo;                  // E 3B4h D=not available, E=error, F=invalid
uint8_t LIM_Stat_Loc_CP;                       // E 3B4h Connector locking 0=unlocked 1=locked 2=error 3=invalid
uint8_t LIM_Stat_Flap_Lock;                      // E 3B4h charging flap 0=unlocked 1=locked 2=error 3=invalid
uint8_t LIM_Chg_Enable;                        // E 3B4h Charging release corresponds to HW line 0=no, 1=yes, 2=error, 3=invalid 
uint8_t LIM_Chg_End_Req;                       // E 3B4h End charging request? 0=no, 1=driver, 2=CSM, 3=invalid
uint8_t LIM_Stat_Unloc_But;                    // E 3B4h Unlocking button only type 1 pressed? 0=no, 1=yes, 3=invalid
uint8_t LIM_Stat_Line_Plg;                     // E 3B4h Cable plugged in? 0=no, 1=yes, 2=error, 3=invalid
uint8_t LIM_ACSE_I_Avbl_Line;                  // E 3B4h Maximum AC current line up to 252A in 1A steps
uint8_t LIM_ACSE_I_Avbl_Grid;                  // E 3B4h Maximum AC current network up to 252A in 1A steps
uint8_t LIM_Chg_AC_Avbl;                       // E 272h Available AC type 0=none, 1=Type1, 2=Type2, 3=China, 7=invalid
uint8_t LIM_Chg_DC_Avbl;                       // E 272h Available DC type 0=none, 1=Chademo, 2=Combo, 3=GB_T, 7=invalid
uint8_t LIM_Stat_DC_Rel;                       // E 272h Indicate CCS contactor state
uint8_t LIM_Stat_Pos_Flap;                           // E 272h Position charging flap 0=closed, 1=open, 2=error, 3=invalid
uint8_t LIM_Alive=0;                           // E 272h Alive counter 0..14

// uint32_t Funk_Netze;                           // S 03Ch 
uint8_t Flap_Ctrl=8;                              // S 2A0h  Charge Port simulator 1=unlock Flap, 2=lock Flap 8=unchanged
uint8_t Ignition_Status_Sim=0x86;                     //simulate ignition status for LIM DC charging 0x86- ignition off, 0x8a- ignition on
uint8_t Flap_Status=1;                            // S 2FCh  Charge Port Flap Status 1=unlocked, 2=locked, F=invalid
uint8_t  DC_Charge_Phase = 0;                    // DC Charging phases 0=standby, 1=init., 2=precharge, 3=energy, 4=switch off, F=invalid
uint32_t ctr_secs_LIMsend_328h=0;                              // seconds counter needed for LIM message 0x328
bool OBD=false;
uint8_t  I_Target_DCCharge = 0;                   //Target current DC charging in A up to 252
uint8_t CCS_Contactor_Ctrl=0;                            //4 bits with DC ccs contactor command. -override: 0=open, 1=open with diag., 2=close
uint8_t  ctr_mins_LIMsend_EOC=0;                           // End of Charge timer: Remaining charging time for the display in minutes
uint16_t ctr_secs_LIMsend_BulkSOC=0;                           //Time to bulk soc target. Remaining charging time up to e.g. 80%
uint16_t ctr_secs_LIMsend_FullSOC=0;                           //Time to full SOC target. Remaining charging time up to 100%
uint8_t CCS_ChargeAmpReq=0;
IntervalTimer timer_10ms_send;                    //interrupt used to send periodic messages
uint8_t ctr_20ms_LIMsend=0;
uint8_t ctr_200ms_LIMsend=0;
uint8_t ctr_100ms_LIMsend=0;
uint8_t ctr_100ms_DRIVEsend=0;
uint8_t ctr_1s_LIMsend=0;
uint8_t ctr_4s_LIMsend=0;
// uint8_t Timer_1Sec=0;
uint8_t Timer_60Sec=0;

// Common
const long canSpeedChas = 500000; // Chassis CAN speed
const long canSpeedElcon = 250000; // Elcon CAN speed
const long canSpeedCHADeMo = 500000; // ChADeMO CAN speed
// uint8_t LIM_SM=0;    //Charge State Machine
uint8_t LIM_SM=0;       //Lim State Machine
bool LIM_SM_flag=false;
uint8_t ChargeStatus = Charge_NotReady;                         // Status_Charging 0=no charging, 1=initial, 2=charging active, 3=charging pause, 4=charging completed, 5=charging error, 15=signal invalid
uint8_t ChargeReady=ChargeReady_No;                          //charge ready flag
uint8_t ChargeReq=ChargeReq_EndCharge;                            //charge request flag
uint32_t ChargePower=0;                         //calculated charge power. 12 bit value scale x25. Values based on 50kw DC fc and 1kw and 3kw ac logs. From bms???
uint8_t ChargeEnd = ChargeEnd_Invalid;
uint8_t DCChargeEnd = DCChargeEnd_Invalid;
bool pbStopChargePressed=false;
bool swIgnition=false;
bool swIgnitionDelayOff=false;
bool swIgnitionDelayOn=false;
// Bounce pbStopCharge = Bounce(inStopCharge,10);                 //10ms debounce button

// PWM
//pump
bool pwmMPumpHigh=false;
bool pwmBPumpHigh=false;
const uint16_t pwmPumpPeriod = 500;
uint16_t pwmMPumpVar = pwmPumpHigh_lowFlow;
uint16_t pwmMPumpVarTemp = pwmMPumpVar;
uint16_t pwmBPumpVar = pwmPumpHigh_lowFlow;
uint16_t pwmBPumpVarTemp = pwmBPumpVar;
//fan
uint16_t pwmMFanReqDuty = pwmFan_low;
uint16_t pwmMFanDutyTemp = pwmMFanReqDuty;
uint16_t pwmBFanReqDuty = pwmFan_low;
uint16_t pwmBFanDutyTemp = pwmBFanReqDuty;
uint16_t pwmFan_freqTemp = pwmFan_freq;

//debug menu
int menuload = 0;
bool simulation_menu = false;
int incomingByte = 0;
bool coolantOverride=false;


void setup() 
{

pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, LOW);
// setup CAN bus**************
// Chassis
delay(1000); // allow CAN hardware to stabilise
Can1.begin();
Can1.setBaudRate(canSpeedChas);
// Can1.setMaxMB(16);
// Can1.enableFIFO();
// Can1.enableFIFOInterrupt();
// Can1.onReceive(canDataReceived);
// Can1.mailboxStatus();
// Elcon
// delay(1000); // allow CAN hardware to stabilise
Can2.begin();
Can2.setBaudRate(canSpeedElcon);
// Can2.setMaxMB(16);
// Can2.enableFIFO();
// Can2.enableFIFOInterrupt();
// Can2.onReceive(canDataReceived);
// Can2.mailboxStatus();
// CHAdeMO
// delay(1000); // allow CAN hardware to stabilise
// Can3.begin();
// Can3.setBaudRate(canSpeedCHADeMo);
// Can2.setMaxMB(16);
// Can2.enableFIFO();
// Can2.enableFIFOInterrupt();
// Can2.onReceive(canDataReceived);
// Can2.mailboxStatus();

// setup PWM 
//pumps
pinMode(pwmOutPumpBat,OUTPUT);
pinMode(pwmOutPumpMain,OUTPUT);
//fans
pinMode(pwmOutFanBat,OUTPUT);
analogWriteFrequency(pwmOutFanBat,pwmFan_freq);
pinMode(pwmOutFanMain,OUTPUT);
analogWriteFrequency(pwmOutFanMain,pwmFan_freq);

// charger Enable to BMS
pinMode(outBMSChargeReq,OUTPUT);
digitalWrite(outBMSChargeReq,LOW);

// stop charging hardware button
pinMode(inStopCharge,INPUT);

// Ignition input
pinMode(inIgnition,INPUT);
//IGNITION SWITCH
if(digitalRead(inIgnition)) {swIgnition=false;}
else {swIgnition=true;}

//charge port flap simulator setup
pinMode(flapSimulatorOut,OUTPUT);
digitalWrite(flapSimulatorOut, HIGH); //open flap

//DCDC enable signal
pinMode(outDCDCenable,OUTPUT);
digitalWrite(outDCDCenable, LOW); //DCDC disabled on boot

//OBC enable signal
pinMode(outOBCenable,OUTPUT);
digitalWrite(outOBCenable, LOW); //OBC disabled on boot

//Zero all CAN message values
Set_ZeroLIM();
Set_ZeroISA();
Set_ZeroBMS();
Set_ZeroDrive();
Set_ZeroElcon();
Set_ZeroIBooster();

// LIM
timer_10ms_send.begin(periodicMSG, 10000);
timer_10ms_send.priority(0);
}

void menu(void)
{
// incomingByte = Serial.read(); // read the incoming byte:

if (incomingByte == 's' && menuload == 0)
  {
    Serial.println();
    Serial.println("MENU");
    Serial.println("Debugging Paused");
    Serial.println("c - Cooling Overrides");
    Serial.println("R - Restart VCU");
    Serial.println("q - exit menu");
    simulation_menu = true;
    menuload = 1;
  }


if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 'R'://restart
        {
          CPU_REBOOT;
          break;
        }

      case 'q': //q to go back to main menu
        {
          menuload = 0;
          simulation_menu = false;
          break;
        }
      
      case 'c': //c for cooling overrides
        {
          while (Serial.available())
          {
            Serial.read();
          }
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println("Cooling Ctrl Menu");
          Serial.println();
          Serial.print("o - override enable toggle: ");
          if (coolantOverride) {Serial.print("enabled");}
          else {Serial.print("disabled");}
          Serial.println();
          Serial.print("B - battery pump speed: ");
          Serial.print(pwmBPumpVarTemp);
          Serial.print("->");
          Serial.print(pwmBPumpVar);
          Serial.println();
          Serial.print("M - motor pump speed: ");
          Serial.print(pwmMPumpVarTemp);
          Serial.print("->");
          Serial.print(pwmMPumpVar);
          Serial.println();
          Serial.print("b - battery fan speed: ");
          Serial.print(pwmBFanDutyTemp );
          Serial.print("->");
          Serial.print(pwmBFanReqDuty);
          Serial.println();
          Serial.print("m - motor fan speed: ");
          Serial.print(pwmMFanDutyTemp );
          Serial.print("->");
          Serial.print(pwmMFanReqDuty);
          Serial.println();
          Serial.print("f - fan base frequency: ");
          Serial.print(pwmFan_freqTemp);
          Serial.println();
          Serial.print("x - Refresh");
          Serial.println();
          Serial.print("q - Go back to menu");
          Serial.println();
          Serial.println();

          menuload = 10;
          break;
        }

      default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

if (menuload == 10)
  {
    switch (incomingByte)
    {
      case 'q': //q to go back to main menu
        {
          menuload = 0;
          incomingByte = 's';
          break;
        }

      case 'x'://refresh 
        {
          menuload = 1;
          incomingByte = 'c';
          break;
        }

      case 'o'://coolant ctrl override toggle 
        {
          coolantOverride=!coolantOverride;
          menuload = 1;
          incomingByte = 'c';
          break;
        }

      case 'B'://battery pump speed
        {
          if (Serial.available() > 0)
          {
            pwmBPumpVarTemp = (uint16_t) (Serial.parseInt());
            pwmBPumpVar=pwmBPumpVarTemp;
            menuload = 1;
            incomingByte = 'c';
          }
          break;
        }

      case 'M'://motor pump speed
        {
          if (Serial.available() > 0)
          {
            pwmMPumpVarTemp = (uint16_t) (Serial.parseInt());
            pwmMPumpVar=pwmMPumpVarTemp;
            menuload = 1;
            incomingByte = 'c';
          }
          break;
        }
      
      case 'b'://battery fan speed
        {
          if (Serial.available() > 0)
          {
            pwmBFanDutyTemp = (uint16_t) (Serial.parseInt());
            pwmBFanReqDuty=pwmBFanDutyTemp ;
            menuload = 1;
            incomingByte = 'c';
          }
          break;
        }

      case 'm'://battery fan speed
        {
          if (Serial.available() > 0)
          {
            pwmMFanDutyTemp = (uint16_t) (Serial.parseInt());
            pwmMFanReqDuty=pwmMFanDutyTemp ;
            menuload = 1;
            incomingByte = 'c';
          }
          break;
        }

      case 'f'://fan base frequency
        {
          if (Serial.available() > 0)
          {
            pwmFan_freqTemp = (uint16_t) (Serial.parseInt());
            analogWrite(pwmOutFanMain,pwmFan_freqTemp);
            menuload = 1;
            incomingByte = 'c';
          }
          break;
        }
      default:
        break;
    }
  }
}


void printFormat(long value,float divider,int decimals)
{
float FloatVal; 
FloatVal = value;
FloatVal = FloatVal/divider;
Serial.print(FloatVal,decimals);  
}

void Debug(void)
{ 
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println(); 
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.print      (" -----------------------------------------------------------VCU -----------------------------------------------------");
    Serial.println(); // Line 0  
    Serial.print("VCU Temperature: ");
    printFormat(VCUtempC,1.0f,2);
    Serial.print("C");
    Serial.println();      
    Serial.print      (" -----------------------------------------------------------Values from BMS, Drive, Ibooster, etc. -----------------------------------------------------");
    Serial.print      (" -----------------------------------------------------------BMS -----------------------------------------------------");
    Serial.println(); // Line 0  
    if (BMS_State == BMS_Boot)         Serial.print ("BMS_Boot| ");  
    if (BMS_State == BMS_Ready)         Serial.print ("BMS_Ready| ");  
    if (BMS_State == BMS_Drive)         Serial.print ("BMS_Drive| ");  
    if (BMS_State == BMS_Charge)         Serial.print ("BMS_Charge| "); 
    if (BMS_State == BMS_Precharge)        Serial.print ("BMS_Precharge| "); 
    if (BMS_State == BMS_Error)         Serial.print ("BMS_Error| "); 
    Serial.println();
    Serial.print("Amps seen by BMS: ");
    printFormat(BMS_BatAmp,10.0f,1);
    Serial.println();
    Serial.print      ("BMS_CellsBal #");
    Serial.print(BMS_CellsBal);
    Serial.println();
    Serial.print      ("BMS Charge Limits: ");
    printFormat(BMS_ChargeVoltLim,10.0f,1);
    Serial.print      ("V ");
    printFormat(BMS_ChargeAmpLim,10.0f,1);
    Serial.print      ("A");
    Serial.println();
    Serial.print      ("BMS disCharge Limits: ");
    printFormat(BMS_disChargeVoltLim,10.0f,1);
    Serial.print      ("V ");
    printFormat(BMS_disChargeAmpLim,10.0f,1);
    Serial.print      ("A");
    Serial.println();
    if (BMS_Error_CellOverVLimit == 0x1) Serial.print ("Error_CellOverVLimit| "); 
    if (BMS_Error_CellUnderVLimit == 0x1) Serial.print ("Error_CellUnderVLimit| "); 
    if (BMS_Error_CellOverTLimit == 0x1) Serial.print ("Error_CellOverTLimit| "); 
    if (BMS_Error_CellUnderTLimit == 0x1) Serial.print ("Error_CellUnderTLimit| "); 
    if (BMS_Error_CellImbalance == 0x1) Serial.print ("Error_CellImbalance| "); 
    if (BMS_Warn_CellOverVLimit == 0x1) Serial.print ("Warn_CellOverVLimit| "); 
    if (BMS_Warn_CellUnderVLimit == 0x1) Serial.print ("Warn_CellUnderVLimit| "); 
    if (BMS_Warn_CellOverTLimit == 0x1) Serial.print ("Warn_CellOverTLimit| ");  
    if (BMS_Warn_CellUnderTLimit == 0x1) Serial.print ("Warn_CellUnderTLimit| ");  
    Serial.println();
    Serial.print      (" -----------------------------------------------------------Motor -----------------------------------------------------");
    Serial.println();
    if (Drive_Dir == Drive_N)         Serial.println ("Drive_N| ");  
    if (Drive_Dir == Drive_F)         Serial.print ("Drive_F| ");  
    if (Drive_Dir == Drive_R)         Serial.print ("Drive_R| ");  
    if (Drive_Dir == Drive_Invalid)   Serial.print ("Drive_Invalid| "); 
    Serial.println();
    // Serial.print (Drive_OpMode,BIN);  
    if (Drive_OpMode == Drive_OPMODE_Off)         Serial.print ("Drive_OPMODE_Off| ");  
    if (Drive_OpMode == Drive_OPMODE_ACHeat)         Serial.print ("Drive_OPMODE_ACHeat| "); 
    if (Drive_OpMode == Drive_OPMODE_Boost)         Serial.print ("Drive_OPMODE_Boost| "); 
    if (Drive_OpMode == Drive_OPMODE_Buck)         Serial.print ("Drive_OPMODE_Buck| "); 
    if (Drive_OpMode == Drive_OPMODE_ManualRun)         Serial.print ("Drive_OPMODE_ManualRun| "); 
    if (Drive_OpMode == Drive_OPMODE_Run)         Serial.print ("Drive_OPMODE_Run| "); 
    if (Drive_OpMode == Drive_OPMODE_Sine)         Serial.print ("Drive_OPMODE_Sine| "); 
    if (Drive_OpMode == Drive_OPMODE_Invalid)         Serial.print ("Drive_OPMODE_Invalid| "); 
    Serial.println();
    // Serial.print (Drive_Status,BIN);  
    if (Drive_Status == Drive_Stat_None)         Serial.print ("Drive_Stat_None| ");
    if ((Drive_Status>>0 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcLow| ");  
    if ((Drive_Status>>1 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcHigh| ");
    if ((Drive_Status>>2 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcBelowUdcSw| ");  
    if ((Drive_Status>>3 & 0x1) == 1)         Serial.print ("Drive_Stat_VdcLim| ");
    if ((Drive_Status>>4 & 0x1) == 1)         Serial.print ("Drive_Stat_EmcyStop| ");  
    if ((Drive_Status>>5 & 0x1) == 1)         Serial.print ("Drive_Stat_MProt| ");
    if ((Drive_Status>>6 & 0x1) == 1)         Serial.print ("Drive_Stat_PotPressed| ");  
    if ((Drive_Status>>7 & 0x1) == 1)         Serial.print ("Drive_Stat_TmpHs| ");
    if ((Drive_Status>>8 & 0x1) == 1)         Serial.print ("Drive_Stat_WaitStart| ");  
    if ((Drive_Status>>9 & 0x1) == 1)         Serial.print ("Drive_Stat_BrakeCheck| ");
    if (Drive_Status == Drive_Stat_Invalid)         Serial.print ("Drive_Stat_Invalid| ");  
    Serial.println();
    printFormat(Drive_MotorTemp,100.0f,2);
    Serial.print      ("C, Drive Motor Temp| ");
    Serial.println();
    printFormat(Drive_HtSnkTemp,100.0f,2); 
    Serial.print      ("C, Drive Inverter/HeatSink Temp| ");
    Serial.println();
    // Serial.print(BMS_CellsTempMax);
    // Serial.print      ("K, BMS_CellsTempMax ");
    // Serial.println();

    // Serial.println();
    // if (pbStopChargePressed)         Serial.print ("btn Stop Charge pressed");
    // else         Serial.print ("btn Stop Charge released");

    Serial.println();
    if (swIgnition)         Serial.print ("swIgnition On");
    else         Serial.print ("swIgnition Off");

    // Serial.println();
    // if (Flap_Status==1)         Serial.print ("Flap_Status=1, unlocked");
    // if (Flap_Status==2)         Serial.print ("Flap_Status=2, locked");
    // if (Flap_Status>2)         Serial.print ("Flap_Status>2, invalid");

    // Serial.println();
    // if (OBD)         Serial.print ("OBD=true, no OBD reset requirement");
    // else         Serial.print ("OBD=false, OBD reset request");

    // Serial.println();
    // Serial.print("Ignition Sim = ");
    // Serial.print(Ignition_Status_Sim,HEX);
    Serial.println();
    Serial.print      (" -----------------------------------------------------------iBooster -----------------------------------------------------");
    Serial.println();
    if (IBST_driverBrakeApply==BRAKES_NOT_APPLIED)         Serial.print ("BRAKES_NOT_APPLIED");
    if (IBST_driverBrakeApply==DRIVER_APPLYING_BRAKES)         Serial.print ("DRIVER_APPLYING_BRAKES");
    if (IBST_driverBrakeApply==FAULT)         Serial.print ("FAULT");
    if (IBST_driverBrakeApply==NOT_INIT_OR_OFF)         Serial.print ("NOT_INIT_OR_OFF");
    if (IBST_driverBrakeApply==IBST_Invalid)         Serial.print ("IBST_Invalid");
    Serial.println();
    if (IBST_iBoosterStatus==IBOOSTER_ACTIVE_GOOD_CHECK)         Serial.print ("IBOOSTER_ACTIVE_GOOD_CHECK");
    if (IBST_iBoosterStatus==IBOOSTER_ACTUATION)         Serial.print ("IBOOSTER_ACTUATION");
    if (IBST_iBoosterStatus==IBOOSTER_DIAGNOSTIC)         Serial.print ("IBOOSTER_DIAGNOSTIC");
    if (IBST_iBoosterStatus==IBOOSTER_FAILURE)         Serial.print ("IBOOSTER_FAILURE");
    if (IBST_iBoosterStatus==IBOOSTER_INIT)         Serial.print ("IBOOSTER_INIT");
    if (IBST_iBoosterStatus==IBOOSTER_OFF)         Serial.print ("IBOOSTER_OFF");
    if (IBST_iBoosterStatus==IBOOSTER_READY)         Serial.print ("IBOOSTER_READY");
    if (IBST_iBoosterStatus==IBST_Invalid)         Serial.print ("IBST_Invalid");
    Serial.println();
    if (IBST_internalState==DIAGNOSTIC)         Serial.print ("DIAGNOSTIC");
    if (IBST_internalState==EXTERNAL_BRAKE_REQUEST)         Serial.print ("EXTERNAL_BRAKE_REQUEST");
    if (IBST_internalState==LOCAL_BRAKE_REQUEST)         Serial.print ("LOCAL_BRAKE_REQUEST");
    if (IBST_internalState==NO_MODE_ACTIVE)         Serial.print ("NO_MODE_ACTIVE");
    if (IBST_internalState==POST_DRIVE_CHECK)         Serial.print ("POST_DRIVE_CHECK");
    if (IBST_internalState==PRE_DRIVE_CHECK)         Serial.print ("PRE_DRIVE_CHECK");
    if (IBST_internalState==TRANSITION_TO_IDLE)         Serial.print ("TRANSITION_TO_IDLE");
    if (IBST_internalState==IBST_Invalid)         Serial.print ("IBST_Invalid");
    Serial.println();
    if(IBST_rodLen<0xffff) Serial.print((double) ((IBST_rodLen*0.015625d)-5.0d),4);
    else Serial.print("NA ");
    Serial.print("mm iBooster Rod");

    Serial.println();
    Serial.print      (" -----------------------------------------------------------Values transmitted to the LIM-----------------------------------------------------");
    Serial.println(); // Line 1
    Serial.print      (" | ");
    printFormat(ISA_BatVolt,10.0f,2);
    Serial.print      ("V ->Bat.Voltage Act.(ISA)| ");
    printFormat(8192-ISA_BatAmp,-10.0f,2);
    Serial.print      ("A ->Bat.Current Act. (ISA)  | ");
    printFormat(ISA_MainPOSVolt,10.0f,2);
    Serial.print      ("V ->MPOS.Voltage Act.(ISA)| ");
    printFormat(ISA_MainNEGVolt,10.0f,2);
    Serial.print      ("V ->MNEG.Voltage Act.(ISA)| ");
    
    // Serial.println(); // Line 2
    // Serial.print      (" | ");
    // printFormat(V_Max_Chg_Bat,10.0f,2);
    // Serial.print      ("V ->Final charging voltage    | ");
    printFormat(I_Target_DCCharge,10.0f,2);
    Serial.print      ("A ->Target charging current        | ");
    // printFormat(Fast_SOC_Tar,2.0f,2);     
    // Serial.print      ("% ->Target SOC fast charging  | "); 
    // printFormat(V_Mot_Current,10.0f,2);
    // Serial.print      ("V ->With.voltage act.     | ");
    
    // Serial.println(); // Line 3
    // Serial.print      (" | ");
    // printFormat(V_Max_Lim,10.0f,2);
    // Serial.print      ("V ->Vehicle voltage Lim. | ");
    // printFormat(I_Max_Lim,1.0f,2);
    // Serial.print      ("A ->Vehicle current Lim.    | ");
    // Serial.print ("                                 | ");
    printFormat(ChargeStatus,1.0f,0);
    Serial.print      ("     ->Charging status to LIM        | ");

    
    Serial.println(); // Line 4
    Serial.print      (" | ");
    printFormat(BMS_CellsVoltMax,1000.0f,3);
    Serial.print      ("V ->Cell voltage Max.   | ");
    printFormat(BMS_CellsVoltMin,1000.0f,3);
    Serial.print      ("V ->Cell voltage Min.   | ");
    // printFormat(BMS_CellsTempMax-273.15,1.0f,3);
    Serial.print(BMS_CellsTempMax-273);
    Serial.print      ("*C ->Cell temp Max.   | ");
    // printFormat(BMS_CellsTempMin-273.15,1.0f,3);
    Serial.print(BMS_CellsTempMin-273);
    Serial.print      ("*C ->Cell temp Min.   | ");
    printFormat(BMS_CapacityAh,1.0f,2);
    Serial.print      ("Ah ->Bat. capacity   | ");
    if (BMS_CapacityWhCharge/10 <=0xffff)
      {
        printFormat(BMS_CapacityWhCharge,10.0f,2);
        Serial.print      ("Wh ->Bat. capacity   | ");
      }
    else
      {
        Serial.print(0xfffe, DEC);
        Serial.print      ("Wh ->Bat. capacity (trunc for LIM)  | ");
      }
    printFormat(LIM_SM,1.0f,0);
    Serial.print      ("     ->Charge SM Step         | ");
    Serial.println();
    Serial.print      (" | ");
    printFormat(BMS_SOC,10.0f,1);
    Serial.print      ("% ->SOC from BMS           | "); 

    
    // Serial.println(); // Line 5
    // Serial.print      (" | ");
    // printFormat(Temp_cell_max-T_off,1.0f,0);
    // Serial.print      ("°C   ->Cell temp. Max.      | ");
    // printFormat(No_Temp_cell_max,1.0f,0);
    // Serial.print      ("     ->Number                | ");
    // printFormat(Temp_cell_min-T_off,1.0f,0);
    // Serial.print      ("°C   ->Cell temp. Min.      | ");
    // printFormat(No_Temp_cell_min,1.0f,0);
    // Serial.print      ("     ->Number                | ");
    
    Serial.println(); // Line 6
    Serial.print      (" | ");
    if (ChargeStatus == 0)         Serial.print ("Charging status: no charging          | ");  
    if (ChargeStatus == 1)         Serial.print ("Charging status: Initializing       | ");  
    if (ChargeStatus == 2)         Serial.print ("Charging status: charging active        | ");  
    if (ChargeStatus == 3)         Serial.print ("Charging status: charging pause            | "); 
    if (ChargeStatus == 4)         Serial.print ("Charging status: Charging completed        | ");  
    if (ChargeStatus == 5)         Serial.print ("Charging status: charging error           | ");  
    if (ChargeStatus == 15)        Serial.print ("Charging status: Signal invalid      | "); 

    if (ChargeReady==ChargeReady_Yes)                Serial.print ("Ready to charge: yes                   | ");  
    else                       Serial.print ("Ready to charge: no                | ");  

    printFormat(ctr_mins_LIMsend_EOC,1.0f,0);
    Serial.print      (" min ->charge remaining time              | ");   
    printFormat(ctr_secs_LIMsend_FullSOC,10.0f,0);
    Serial.print      (" sec ->charge remaining time to FULL              | "); 
    printFormat(ctr_secs_LIMsend_BulkSOC,10.0f,0);
    Serial.print      (" sec ->charge remaining time to BULK(80%)              | ");           
                

    Serial.println();
    Serial.print      (" -----------------------------------------------------------------Values received from the LIM----------------------------------------------------");
    Serial.println(); // Line 1
    Serial.print      (" | ");
    if (LIM_Chg_AC_Avbl == 0)         Serial.print ("AC Available no                | ");  
    if (LIM_Chg_AC_Avbl == 1)         Serial.print ("AC Available Typ1                | ");  
    if (LIM_Chg_AC_Avbl == 2)         Serial.print ("AC Available Typ2                | ");  
    if (LIM_Chg_AC_Avbl == 3)         Serial.print ("AC Available China               | "); 
    if (LIM_Chg_AC_Avbl == 7)         Serial.print ("AC Available invalid        | "); 

    if (LIM_Chg_DC_Avbl == 0)         Serial.print ("DC Available no                | "); 
    if (LIM_Chg_DC_Avbl == 1)         Serial.print ("DC Available Chademo             | "); 
    if (LIM_Chg_DC_Avbl == 2)         Serial.print ("DC Available Combo               | "); 
    if (LIM_Chg_DC_Avbl == 3)         Serial.print ("DC Available GB T                | ");
    if (LIM_Chg_DC_Avbl == 7)         Serial.print ("DC Available invalid        | "); 
    
    if (LIM_V_CP_DC/2 < 251)
      {
      printFormat(LIM_V_CP_DC,0.5f,2); 
      Serial.print    ("V ->DC voltage at the plug| ");
      }
    else
      {                                 
        if (LIM_V_CP_DC/2==251)Serial.print ("AC or Neg. Voltage at the plug | ");
        if (LIM_V_CP_DC/2==252)Serial.print ("Voltage measurement inactive   | ");
        if (LIM_V_CP_DC/2==253)Serial.print ("Voltage measurement not available.| ");
        if (LIM_V_CP_DC/2==254)Serial.print ("Voltage measurement error     | ");
        if (LIM_V_CP_DC/2==255)Serial.print ("Voltage measurement invalid  | ");
      }

    printFormat(LIM_Alive,1.0f,0); 
    Serial.print    ("      LIM Alive              | ");  
    

    Serial.println(); // Line 2
    Serial.print      (" | ");

    if (LIM_Stat_Hook_Pos == 0)   Serial.print ("Inlet hook position no | "); 
    if (LIM_Stat_Hook_Pos == 1)   Serial.print ("Inlet hook position yes   | ");
    if (LIM_Stat_Hook_Pos == 3)  Serial.print ("Inlet hook position invalid | ");
    
    if (LIM_Stat_Hook_Loc == 0)   Serial.print ("Inlet lock position no | ");
    if (LIM_Stat_Hook_Loc == 1)   Serial.print ("Inlet lock position yes   | ");
    if (LIM_Stat_Hook_Loc == 3)  Serial.print ("Inlet lock position invalid | ");

    if (LIM_ACSE_I_Avbl_Line < 253) printFormat(LIM_ACSE_I_Avbl_Line,1.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("A ->Maximum current AC cable| ");
    
    if (LIM_ACSE_I_Avbl_Grid < 253) printFormat(LIM_ACSE_I_Avbl_Grid,1.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("A ->Maximum current available at the charger | ");
    
    if (LIM_Stat_Pos_Flap == 0)             Serial.print ("flap closed                    | ");
    if (LIM_Stat_Pos_Flap == 1)             Serial.print ("flap open                   | ");
    if (LIM_Stat_Pos_Flap == 2)             Serial.print ("flap position error      | ");
    if (LIM_Stat_Pos_Flap == 3)             Serial.print ("flap postion invalid     | ");

    if (LIM_Stat_Flap_Lock == 0)        Serial.print ("flap unlocked              | ");
    if (LIM_Stat_Flap_Lock == 1)        Serial.print ("flap locked               | ");
    if (LIM_Stat_Flap_Lock == 2)        Serial.print ("flap lock error       | ");
    if (LIM_Stat_Flap_Lock == 3)        Serial.print ("flap lock invalid     | ");
    
    Serial.println();   // Line 3
    Serial.print      (" | ");
    if (LIM_Stat_Line_Plg == 0)       Serial.print ("Cable plugged in no              | ");
    if (LIM_Stat_Line_Plg == 1)       Serial.print ("Cable plugged in yes                | ");
    if (LIM_Stat_Line_Plg == 2)       Serial.print ("Cable plugged in error            | ");
    if (LIM_Stat_Line_Plg == 3)       Serial.print ("Cable plugged in invalid              | ");

    if (LIM_Stat_Pilot == 0)          Serial.print ("Pilot none                     | ");
    if (LIM_Stat_Pilot == 1)          Serial.print ("Pilot 10-95% not ready      | ");
    if (LIM_Stat_Pilot == 2)          Serial.print ("Pilot 10-95% ready              | ");
    if (LIM_Stat_Pilot == 3)          Serial.print ("Pilot error                     | ");
    if (LIM_Stat_Pilot == 4)          Serial.print ("Pilot 5% not ready            | ");
    if (LIM_Stat_Pilot == 5)          Serial.print ("Pilot 5% ready                  | "); 
    if (LIM_Stat_Pilot == 6)          Serial.print ("Pilot static                   | ");
    if (LIM_Stat_Pilot == 7)          Serial.print ("Pilot invalid                   | ");

    if (LIM_Charger_Type == 0)         Serial.print ("Current charging type none         | ");
    if (LIM_Charger_Type == 1)         Serial.print ("Current charging type AC-Typ1        | ");
    if (LIM_Charger_Type == 2)         Serial.print ("Current charging type AC-Typ2        | ");
    if (LIM_Charger_Type == 3)         Serial.print ("Current charging type Chahdemo       | ");
    if (LIM_Charger_Type == 4)         Serial.print ("Current charging type DC-Typ1        | ");
    if (LIM_Charger_Type == 5)         Serial.print ("Current charging type AC-CN          | ");
    if (LIM_Charger_Type == 6)         Serial.print ("Current charging type AC-Combo1       | ");
    if (LIM_Charger_Type == 7)         Serial.print ("Current charging type AC-Combo2      | ");
    if (LIM_Charger_Type == 8)         Serial.print ("Current charging type DC-Typ2        | ");
    if (LIM_Charger_Type == 9)         Serial.print ("Current charging type DC-Combo2      | ");
    if (LIM_Charger_Type ==10)         Serial.print ("Current charging type DC-GB_T        | ");
    if (LIM_Charger_Type ==0xFF)        Serial.print ("Current charging type invalid       | ");
    
    // printFormat(SAM_Neu,1.0f,0);
    // Serial.print    ("      SAM counter             | ");  
    // //if (LIM_Stat_Hospitality == 0)    Serial.print ("Hospitality aus                  | ");
    // //if (LIM_Stat_Hospitality == 1)    Serial.print ("Hospitality ein                  | ");
    // //if (LIM_Stat_Hospitality == 15)   Serial.print ("Hospitality ung.                 | ");

    Serial.println(); // Line 4
    Serial.print      (" | ");

    if (LIM_Stat_Loc_CP == 0)         Serial.print ("Plug unlocked              | ");
    if (LIM_Stat_Loc_CP == 1)         Serial.print ("Plug locked               | ");
    if (LIM_Stat_Loc_CP == 2)         Serial.print ("Plug unlocked error             | ");
    if (LIM_Stat_Loc_CP == 3)         Serial.print ("Plug unlocked invalid              | ");
    
    if (LIM_Stat_Line_Held == 0)      Serial.print ("Charging line held no        | ");
    if (LIM_Stat_Line_Held == 1)      Serial.print ("Charging line held when parking | ");
    if (LIM_Stat_Line_Held == 2)      Serial.print ("Charging line held error      | ");
    if (LIM_Stat_Line_Held == 3)      Serial.print ("Charging line held invalid    | ");

    /*
    if (LIM_Stat_Line_Chademo == 0) Serial.print ("Chademo gesteckt no   | ");
    if (LIM_Stat_Line_Chademo == 1) Serial.print ("Chademo gesteckt yes     | ");
    if (LIM_Stat_Line_Chademo == 2) Serial.print ("Chademo gesteckt fehler | ");
    if (LIM_Stat_Line_Chademo == 3) Serial.print ("Chademo gesteckt ung.   | ");

    if (LIM_Stat_Crt_Chademo == 0xD) Serial.print ("Chademo Crt nicht verv. | ");
    if (LIM_Stat_Crt_Chademo == 0xE) Serial.print ("Chademo Crt fehler      | ");
    if (LIM_Stat_Crt_Chademo == 0xF) Serial.print ("Chademo Crt unültig     | ");
*/
    if (LIM_Stat_Unloc_But == 0)      Serial.print ("Type1 unlocking pressed? no       | ");
    if (LIM_Stat_Unloc_But == 1)      Serial.print ("Type1 unlocking pressed? yes         | ");
    if (LIM_Stat_Unloc_But == 3)      Serial.print ("Type1 unlocking pressed? invalid       | ");
    
    

    if (LIM_Chg_Enable == 0)          Serial.print ("LIM Charge enable? no                | ");
    if (LIM_Chg_Enable == 1)          Serial.print ("LIM Charge enable? yes                  | ");
    if (LIM_Chg_Enable == 2)          Serial.print ("LIM Charge enable? error              | ");
    if (LIM_Chg_Enable == 3)          Serial.print ("LIM Charge enable? invalid            | ");
    
    if (LIM_Chg_End_Req == 0)         Serial.print ("End charging request no       | ");
    if (LIM_Chg_End_Req == 1)         Serial.print ("End charging request Driver     | ");
    if (LIM_Chg_End_Req == 2)         Serial.print ("End charging request CSM        | ");
    if (LIM_Chg_End_Req == 3)         Serial.print ("End charging request invalid   | ");
    
    Serial.println();  // Line 5
    Serial.print      (" | ");
    if (LIM_Stat_DC_Rel==0x3F)        Serial.print ("CCS contactor state: invalid                 |"); 
    if (LIM_Stat_DC_Rel==0x3E)        Serial.print ("CCS contactor state: error                   |"); 
    // if (LIM_Stat_DC_Rel==0x3E)        Serial.print ("DC-Rel. not verifiable                  |");  
    if (LIM_Stat_DC_Rel< 0x20)
      {
      if (((LIM_Stat_DC_Rel>>0)&1)==0) Serial.print ("at least one CCS/DC contactor is opened | "); 
      else                             Serial.print ("Both CCS/DC contactors closed      | "); 
      if (((LIM_Stat_DC_Rel>>1)&1)==0) Serial.print ("CCS/DC Contactor control OK            | "); 
      else                             Serial.print ("CCS/DC Contactor control. not OK     | "); 
      if (((LIM_Stat_DC_Rel>>2)&7)==0) Serial.print ("CCS/DC Contactor OK                       | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==1) Serial.print ("CCS/DC Contactor no diagnosis yet.  | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==2) Serial.print ("CCS/DC Contactor diagnosis is running            | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==3) Serial.print ("CCS/DC Contactor diagnosis not possible  | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==4) Serial.print ("A CCS/DC Contactor stuck              | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==5) Serial.print ("Both CCS/DC Contactors glued together            | ");
      if (((LIM_Stat_DC_Rel>>2)&7)==6) Serial.print ("Unexpected DC on the plug       | "); 
      if (((LIM_Stat_DC_Rel>>2)&7)==7) Serial.print ("Both CCS/DC Contactor are not discounted. DC a. S. | ");
      }
    
      // if (digitalRead (DC_Pre))                        Serial.print ("DC_Pre   ein                    | ");  
      // else                              Serial.print ("DC_Pre   aus                    | "); 
    // if (LIM_Stat_Pilot == 1 || LIM_Stat_Pilot == 2) 
    Debug_AC(); 
    if (LIM_Stat_Pilot == 4 || LIM_Stat_Pilot == 5 || LIM_Stat_Pilot == 6) {Debug_DC();}
    Serial.println("");
    Serial.println(" ----------------------------------------------------END---------------------------------------------");
    Debug_Stamp = millis();
}

void Debug_AC(void)
{
    Serial.println();  
    Serial.print      (" ----------------------------------------------------Values sent to the Elcon---------------------------------------------");
    Serial.println(); 
    Serial.print      (" ----------------------------------------------------AC Charger---------------------------------------------");
    Serial.println();   
    if (BMS_ChargeVoltLim < 0xFFFF) printFormat(BMS_ChargeVoltLim,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("VDC ->AC Charger Limit Volts        | ");
    // Serial.println();   
    if (BMS_ChargeAmpLim < 0xFFFF) printFormat(ElconCharger_AmpReq,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("ADC ->AC Charger Limit Amps        | ");
    Serial.println();  
    Serial.print      (" ----------------------------------------------------Values received from the Elcon---------------------------------------------");
    Serial.println(); 
    Serial.print      (" ----------------------------------------------------AC Charger---------------------------------------------");
    Serial.println();
    if (ElconCharger_VoltOutput < 0xFFFF) printFormat(ElconCharger_VoltOutput,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("V ->DC Volt Output         | ");
    
    // Serial.println();
    if (ElconCharger_AmpOutput < 0xFFFF) printFormat(ElconCharger_AmpOutput,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("A ->DC Amp Output         | ");

    // Serial.println();
    if (ElconCharger_Temp < 0xFFFF) printFormat(ElconCharger_Temp,1.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("C Temp        | ");

    Serial.println();
    if (ElconCharger_CommTimeout==1) Serial.print ("Comms Timeout");
    else if (ElconCharger_CommTimeout==0) Serial.print ("Comms OK");

    Serial.print(" | ");
    if (ElconCharger_HardwareError==1) Serial.print ("Hardware Error");
    else if (ElconCharger_HardwareError==0) Serial.print ("Hardware OK");

    Serial.print(" | ");
    if (ElconCharger_TempError==1) Serial.print ("Temp Error");
    else if (ElconCharger_TempError==0) Serial.print ("Temp OK");

    Serial.println();
    if (ElconCharger_InVoltError==1) Serial.print ("Input Voltage Error");
    else if (ElconCharger_InVoltError==0) Serial.print ("Input Voltage OK");

    Serial.print(" | ");
    if (ElconCharger_BatVoltError==1) Serial.print ("Battery Volt Error");
    else if (ElconCharger_BatVoltError==0) Serial.print ("Battery Volt OK");

    Serial.print(" | ");
    if (ElconCharger_msgInvalid) Serial.print ("Invalid CAN message");
    else Serial.print ("CAN message OK");

    Serial.println(); 
    Serial.print      (" ----------------------------------------------------DCDC---------------------------------------------");
    Serial.println();
    if (ElconDCDC_VoltOutput < 0xFFFF) printFormat(ElconDCDC_VoltOutput,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("V ->DC Volt Output         | ");

    // Serial.println();
    if (ElconDCDC_AmpOutput < 0xFFFF) printFormat(ElconDCDC_AmpOutput,10.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("A ->DC Amp Output         | ");

    // Serial.println();
    if (ElconDCDC_Temp < 0xFFFF) printFormat(ElconDCDC_Temp,1.0f,2);
    else Serial.print ("   -.--");
    Serial.print      ("C Temp        | ");

    Serial.println();
    if (ElconDCDC_HVILError==1) Serial.print ("HVIL Error");
    else if (ElconDCDC_HVILError==0) Serial.print ("HVIL OK");
    Serial.print(" | ");
    if (ElconDCDC_WaterFanSts==1) Serial.print ("WaterFanSts ON");
    else if (ElconDCDC_WaterFanSts==0) Serial.print ("WaterFanSts OFF");
    Serial.print(" | ");
    if (ElconDCDC_StopError==1) Serial.print ("Stop Error");
    else if (ElconDCDC_StopError==0) Serial.print ("Stop OK");
    Serial.println();
    if (ElconDCDC_WaterFanSts2==1) Serial.print ("WaterFanSts2 ON");
    else if (ElconDCDC_WaterFanSts2==0) Serial.print ("WaterFanSts2 OFF");
    Serial.print(" | ");
    if (ElconDCDC_CommTimeout==1) Serial.print ("Comms Timeout");
    else if (ElconDCDC_CommTimeout==0) Serial.print ("Comms OK");
    Serial.print(" | ");
    if (ElconDCDC_HardwareError==1) Serial.print ("Hardware Error");
    else if (ElconDCDC_HardwareError==0) Serial.print ("Hardware OK");
    Serial.println();
    if (ElconDCDC_Sts==1) Serial.print ("Working");
    else if (ElconDCDC_Sts==0) Serial.print ("Stopped");
    Serial.print(" | ");
    if (ElconDCDC_Ready==1) Serial.print ("Ready");
    else if (ElconDCDC_Ready==0) Serial.print ("Initialising");
    Serial.print(" | ");
    if (ElconDCDC_OutOverAmp==1) Serial.print ("Output OverAmp Error");
    else if (ElconDCDC_OutOverAmp==0) Serial.print ("Output Amp OK");
    Serial.println();
    if (ElconDCDC_InUnderVolt==1) {Serial.print ("Input Under Volt Error");Serial.print(" | ");}
    if (ElconDCDC_InOverVolt==1) {Serial.print ("Input Over Volt Error");Serial.print(" | ");}
    if (ElconDCDC_InUnderVolt==0 and ElconDCDC_InOverVolt==0) {Serial.print ("Input Volt OK");Serial.print(" | ");}
    if (ElconDCDC_OutUnderVolt==1) {Serial.print ("Output Under Volt Error");Serial.print(" | ");}
    if (ElconDCDC_OutOverVolt==1) {Serial.print ("Output Over Volt Error");Serial.print(" | ");}
    if (ElconDCDC_OutUnderVolt==0 && ElconDCDC_OutOverVolt==0) {Serial.print ("Output Volt OK");Serial.print(" | ");}
    if (ElconDCDC_OverTemp==1) {Serial.print ("Temp Error");Serial.print(" | ");}
    if (ElconDCDC_HighTemp==1) {Serial.print ("Temp Warning");Serial.print(" | ");}
    if (ElconDCDC_OverTemp==0 && ElconDCDC_HighTemp==0) {Serial.print ("Temp OK");Serial.print(" | ");}
    Serial.println();
    if (ElconDCDC_msgInvalid) Serial.print ("Invalid CAN message");
    else Serial.print ("CAN message OK");
}

void Debug_DC(void)
{
  Serial.println();  
  Serial.print      (" -------------------------------------------------------Values received from the LIM during DC charging------------------------------------------------");
  Serial.println(); //Line 1
  Serial.print      (" | ");
  if (LIM_DCSE_V_Avbl < 0xFFFF) printFormat(LIM_DCSE_V_Avbl,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("V ->Available clamps Max | ");
  if (LIM_DCSE_I_Avbl < 0xFFFF) printFormat(LIM_DCSE_I_Avbl,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("A ->Available power max | ");
  if (LIM_DCSE_V_Trhd < 0xFFFF) printFormat(LIM_DCSE_V_Trhd,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("V ->Threshold voltage     | ");
  if (LIM_DCSE_Prot_Num < 0xFF) printFormat(LIM_DCSE_Prot_Num,1.0f,0);
  else Serial.print ("   -");
  Serial.print      ("     ->Protocol number       | ");

  Serial.println(); // Line 2
  Serial.print      (" | ");
  if (LIM_DCSE_V_Min_Avbl < 0xFFFF) printFormat(LIM_DCSE_V_Min_Avbl,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("V ->Available clamps min | ");
  if (LIM_DCSE_I_Min_Avbl < 0xFFF) printFormat(LIM_DCSE_I_Min_Avbl,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("A ->Available electricity min | ");
  if (LIM_DCSE_I_Tol < 0xFFF) printFormat(LIM_DCSE_I_Tol,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("A ->Current tolerance       | ");
  if (LIM_DCSE_I_Riple < 0xFFF) printFormat(LIM_DCSE_I_Riple,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("A ->Current ripple      | ");

  Serial.println(); // Line 3
  Serial.print      (" | ");
  if (LIM_DCSE_Stat_Chg==0)  Serial.print ("Status: not ready            | ");
  if (LIM_DCSE_Stat_Chg==1)  Serial.print ("Status: ready                   | ");
  if (LIM_DCSE_Stat_Chg==2)  Serial.print ("Status: turn off              | ");
  if (LIM_DCSE_Stat_Chg==3)  Serial.print ("Status: interrupt             | ");
  if (LIM_DCSE_Stat_Chg==4)  Serial.print ("Status: preload                 | ");
  if (LIM_DCSE_Stat_Chg==5)  Serial.print ("Status: Isolations Monitoring    | ");
  if (LIM_DCSE_Stat_Chg==6)  Serial.print ("Status: Emergency stop switch                   | "); 
  if (LIM_DCSE_Stat_Chg==7)  Serial.print ("Status: Malfunction             | ");
  if (LIM_DCSE_Stat_Chg==15) Serial.print ("Status: Signal invalid          | ");

  if (LIM_DCSE_Stat_Iso==0)  Serial.print ("Isolation invalid               | ");
  if (LIM_DCSE_Stat_Iso==1)  Serial.print ("Isolation valid                 | ");
  if (LIM_DCSE_Stat_Iso==2)  Serial.print ("Isolation error                 | ");
  if (LIM_DCSE_Stat_Iso==3)  Serial.print ("Isolation signal invalid        | ");

  if (LIM_DCSE_Stat_Iso==0)  Serial.print ("Weld Detection available? no   | ");
  if (LIM_DCSE_Stat_Iso==1)  Serial.print ("Weld Detection available? yes     | ");
  if (LIM_DCSE_Stat_Iso==3)  Serial.print ("Weld Detection available? invalid.   | ");

  Serial.println(); // Line 4
  Serial.print      (" | ");
  if (LIM_DCSE_Eng_Trsm < 0xFF) printFormat(LIM_DCSE_Eng_Trsm,4.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("kW->transmitted by EVSE   | ");
    
  if (LIM_DCSE_Location==0)  Serial.print ("Location private                  | ");
  if (LIM_DCSE_Location==1)  Serial.print ("Office location                    | ");
  if (LIM_DCSE_Location==2)  Serial.print ("Location public              | ");
  if (LIM_DCSE_Location==3)  Serial.print ("Location invalid                | ");

  if (LIM_DCSE_P_LIM_Reached==0)  Serial.print ("Power limit reached? no   | ");
  if (LIM_DCSE_P_LIM_Reached==1)  Serial.print ("Power limit reached? yes     | ");
  if (LIM_DCSE_P_LIM_Reached==3)  Serial.print ("Power limit reached? invalid   | ");

  Serial.println(); // Line 5
  Serial.print      (" | ");
  if (LIM_DCSE_V_Current < 0xFFFF) printFormat(LIM_DCSE_V_Current,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("V ->Actual voltage from EVSE | ");
  if (LIM_DCSE_I_Current < 0xFFFF) printFormat(LIM_DCSE_I_Current,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("A ->Actual current from EVSE   | ");
  if (LIM_DCSE_Rst_Tme_Chg < 0xFFFF) printFormat(LIM_DCSE_Rst_Tme_Chg,10.0f,2);
  else Serial.print ("   -.--");
  Serial.print      ("s ->EVSE remaining charging time | ");

  Serial.println(); // Line 6
  Serial.print      (" | ");
  if (LIM_DCSE_V_LIM_Reached==0)  Serial.print ("Voltage limit reached? no   | ");
  if (LIM_DCSE_V_LIM_Reached==1)  Serial.print ("Voltage limit reached? yes     | ");
  if (LIM_DCSE_V_LIM_Reached==3)  Serial.print ("Voltage limit reached? invalid   | ");

  if (LIM_DCSE_I_LIM_Reached==0)  Serial.print ("Current limit reached? no       | ");
  if (LIM_DCSE_I_LIM_Reached==1)  Serial.print ("Current limit reached? yes         | ");
  if (LIM_DCSE_I_LIM_Reached==3)  Serial.print ("Current limit reached? invalid   | ");

  if (LIM_DCSE_DC_Stp_Crt==0)  Serial.print ("DC charging stop tracking mode       | ");
  if (LIM_DCSE_DC_Stp_Crt==1)  Serial.print ("DC charging stop suppression mode | ");
  if (LIM_DCSE_DC_Stp_Crt==3)  Serial.print ("DC charging stop invalid              | ");

  if (LIM_DCSE_Bat_Comp==0)    Serial.print ("Battery compatible? yes          | ");
  if (LIM_DCSE_Bat_Comp==1)    Serial.print ("Battery compatible? no        | ");

  Serial.println(); // Line 7
  Serial.print      (" | ");
  if (LIM_DCSE_Bat_Prob==0)    Serial.print ("Battery malfunction? no      | ");
  if (LIM_DCSE_Bat_Prob==1)    Serial.print ("Battery malfunction? yes        | ");
  if (LIM_DCSE_Bat_Prob==3)    Serial.print ("Battery malfunction? invalid  | ");

  if (LIM_DCSE_Loc_Stat==0)    Serial.print ("Locked? no                 | ");
  if (LIM_DCSE_Loc_Stat==1)    Serial.print ("Locked? yes                   | ");
  if (LIM_DCSE_Loc_Stat==3)    Serial.print ("Locked? invalid             | ");

  if (LIM_DCSE_Chg_Prob==0)    Serial.print ("Charging malfunction? no         | ");
  if (LIM_DCSE_Chg_Prob==1)    Serial.print ("Charging malfunction? yes           | ");
  if (LIM_DCSE_Chg_Prob==3)    Serial.print ("Charging malfunction? invalid     | ");

  if (LIM_DCSE_Chg_Stat==0)    Serial.print ("charging status: standby            | ");
  if (LIM_DCSE_Chg_Stat==1)    Serial.print ("charging status: active              | ");
  if (LIM_DCSE_Chg_Stat==3)    Serial.print ("charging status: invalid           | ");
} 
// Zeroing local vars
void Set_ZeroISA(void)
{
  ISA_BatVolt = 0xFFFF; 
  ISA_MainPOSVolt = 0xFFFF;
  ISA_BatAmp = 0xFFFF;
  ISA_MainNEGVolt=0;
}
void Set_ZeroIBooster(void)
{
  IBST_driverBrakeApply=IBST_Invalid;
  IBST_iBoosterStatus=IBST_Invalid;
  IBST_internalState=IBST_Invalid;
  IBST_rodLen=0xffff;
}
void Set_ZeroElcon(void)
{
  ElconCharger_VoltOutput=0;
  ElconCharger_AmpOutput=0;
  ElconCharger_Temp=0;
  ElconCharger_HardwareError=0xff;
  ElconCharger_TempError=0xff;
  ElconCharger_InVoltError=0xff;
  ElconCharger_BatVoltError=0xff;
  ElconCharger_CommTimeout=0xff;
  ElconCharger_msgInvalid=false;
  ElconDCDC_VoltOutput=0; //0.1V/bit Offset:0
  ElconDCDC_AmpOutput=0; //0.1V/bit Offset:0
  ElconDCDC_HVILError=0xff; //0:Lock accomplish,1Non-Lock
  ElconDCDC_WaterFanSts=0xff; //0:FAN OFF。1: FAN ON
  ElconDCDC_StopError=0xff; //1:Error, 0:No error
  ElconDCDC_WaterFanSts2=0xff; //0:off。1:ON
  ElconDCDC_CommTimeout=0xff; //1:Error, 0:No error
  ElconDCDC_HardwareError=0xff; //1:Error, 0:No error
  ElconDCDC_Sts=0xff; //1:working, 0:stopped
  ElconDCDC_Ready=0xff; //initialisation 0:uncompleted; 1:completed
  ElconDCDC_OutOverAmp=0xff; //1:Error, 0:No error
  ElconDCDC_InUnderVolt=0xff; //1:Error, 0:No error
  ElconDCDC_InOverVolt=0xff; //1:Error, 0:No error
  ElconDCDC_OutUnderVolt=0xff; //1:Error, 0:No error
  ElconDCDC_OutOverVolt=0xff; //1:Error, 0:No error
  ElconDCDC_OverTemp=0xff; //1:Error, 0:No error
  ElconDCDC_HighTemp=0xff; ////1:High temp, 0:Normal temp
  ElconDCDC_msgInvalid=false;
  ElconDCDC_Temp=0;
}
void Set_ZeroLIM(void)
{
  LIM_Stat_Hook_Pos=3;                 
  LIM_Stat_Hook_Loc=3;                 
  LIM_Stat_Hospitality=15;             
  LIM_V_CP_DC=0xFFFF;                    
  LIM_Charger_Type=0xFF;                
  LIM_Stat_Line_Held=3;                
  LIM_Stat_Line_Chademo=3;             
  LIM_Stat_Pilot=7;                    
  LIM_Stat_Crt_Chademo=15;             
  LIM_Stat_Loc_CP=3;                   
  LIM_Stat_Flap_Lock=3;                  
  LIM_Chg_Enable=3;                     
  LIM_Chg_End_Req=3;                   
  LIM_Stat_Unloc_But=3;                
  LIM_Stat_Line_Plg=3;                 
  LIM_ACSE_I_Avbl_Line=0xFF;           
  LIM_ACSE_I_Avbl_Grid=0xFF;           
  LIM_Chg_AC_Avbl=0b111;               
  LIM_Chg_DC_Avbl=0b111;               
  LIM_Stat_DC_Rel=0b111111;            
  LIM_Stat_Pos_Flap=3;        
  Set_ZeroDC();               
}
void Set_ZeroDC(void)
{
  LIM_DCSE_Prot_Num=0xFF;
  LIM_DCSE_V_Trhd=0xFFFF;
  LIM_DCSE_I_Avbl=0xFFFF;
  LIM_DCSE_V_Avbl=0xFFFF;
  LIM_DCSE_Stat_Iso=3;
  LIM_DCSE_Stat_Chg=15;
  LIM_DCSE_Weld_Det=3;

  LIM_DCSE_Rst_Tme_Chg=0xFFFF;
  LIM_DCSE_V_LIM_Reached=3;
  LIM_DCSE_I_LIM_Reached=3;
  LIM_DCSE_DC_Stp_Crt=3;
  LIM_DCSE_Bat_Prob=3;
  LIM_DCSE_Bat_Comp=3;
  LIM_DCSE_Loc_Stat=3;
  LIM_DCSE_Chg_Prob=3;
  LIM_DCSE_Chg_Stat=3;
  LIM_DCSE_I_Current=0xFFFF;
  LIM_DCSE_V_Current=0xFFFF;

  LIM_DCSE_Eng_Trsm=0xFF;
  LIM_DCSE_Location=3;
  LIM_DCSE_P_LIM_Reached=3;
  LIM_DCSE_I_Tol=0XFFF;
  LIM_DCSE_I_Riple=0XFFF;
  LIM_DCSE_I_Min_Avbl=0xFFF;
  LIM_DCSE_V_Min_Avbl=0xFFFF;
}

void Set_ZeroBMS(void)
{
  BMS_State = BMS_Error; //unknown state
  BMS_ChargeVoltLim=0;
  BMS_ChargeAmpLim=0;
  BMS_disChargeVoltLim=0;
  BMS_disChargeAmpLim=0;
  BMS_SOC=0;
  BMS_CapacityAh=0;
  BMS_CapacityWhCharge=0;
  BMS_CellsVoltMin = 0;
  BMS_CellsVoltMax = 0;
  BMS_CellsTempMin = 0;
  BMS_CellsTempMax = 0;
  BMS_PackVolt=0;
  BMS_AvgTemp=0;
  BMS_CellsBal=0xff;
  BMS_Error_CellOverVLimit = 0xff; //alarm: cell over high Voltage setpoint
  BMS_Error_CellUnderVLimit = 0xff; //alarm: cell under low Voltage setpoint
  BMS_Error_CellOverTLimit = 0xff; //alarm: cell over high Temperature setpoint
  BMS_Error_CellUnderTLimit = 0xff; //alarm: cell under low Temperature setpoint
  BMS_Error_CellImbalance = 0xff; //alarm: cell voltage imbalance
  BMS_Warn_CellOverVLimit = 0xff; //warning: cell over high Voltage setpoint
  BMS_Warn_CellUnderVLimit = 0xff; //warning: cell under low Voltage setpoint
  BMS_Warn_CellOverTLimit = 0xff; //warning: cell over high Temperature setpoint
  BMS_Warn_CellUnderTLimit = 0xff; //warning: cell under low Temperature setpoint
}
void Set_ZeroDrive(void)
{
  Drive_Dir=Drive_Invalid;
  Drive_OpMode=Drive_OPMODE_Invalid;
  Drive_Status=Drive_Stat_Invalid;
  Drive_MotorTemp=0xFFFF;
  Drive_HtSnkTemp=0xFFFF;
}

// Decoding checks
void Check_ISA(CAN_message_t incoming)
{
  switch (incoming.id)
      {
        case canID_ISAmilliAmps:
        {
          long readingISAamps=0;
          // bigE
          readingISAamps = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
          // littleE
          // readingISAamps = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
          readingISAamps=readingISAamps/100; //miliamps to amps in 0.1A steps
          readingISAamps=8192-readingISAamps;
          if (readingISAamps > 0) ISA_BatAmp=(uint16_t)readingISAamps;
          else ISA_BatAmp = 0;
          break;
        }
        case canID_ISAVolt1:
        {
          long readingISAVolt1=0;
          // bigE
          readingISAVolt1 = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
          // littleE
          // readingISAVolt1 = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
          readingISAVolt1=readingISAVolt1/100; //milivolts to volts in 0.1V steps
          if (readingISAVolt1 > 0) ISA_BatVolt=readingISAVolt1;
          else ISA_BatVolt = 0;
          break;
        }
        case canID_ISAVolt3:
        {
          long readingISAVolt3=0;
          // bigE
          readingISAVolt3 = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
          // littleE
          // readingISAVolt3 = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
          readingISAVolt3=readingISAVolt3/100; //milivolts to volts in 0.1V steps
          if (readingISAVolt3 > 0) ISA_MainNEGVolt=readingISAVolt3;
          else ISA_MainNEGVolt = 0;
          break;
        }
        case canID_ISAVolt2:
        {
          long readingISAVolt2=0;
          // bigE
          readingISAVolt2 = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
          // littleE
          // readingISAVolt2 = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
          readingISAVolt2=readingISAVolt2/100; //milivolts to volts in 0.1V steps
          readingISAVolt2=readingISAVolt2-ISA_MainNEGVolt;
          if (readingISAVolt2 > 0) ISA_MainPOSVolt=readingISAVolt2;
          else ISA_MainPOSVolt = 0;
          break;
        }
        // case canID_ISAsoc:
        // {
        //   long readingISASoC=0;
        //   // bigE
        //   readingISASoC = (long)((incoming.buf[2] << 24) | (incoming.buf[3] << 16) | (incoming.buf[4] << 8) | (incoming.buf[5]));
        //   // littleE
        //   // readingISASoC = (long)((incoming.buf[5] << 24) | (incoming.buf[4] << 16) | (incoming.buf[3] << 8) | (incoming.buf[2]));
        //   readingISASoC=readingISASoC*10; //convert to SoC in 0.1Wh steps
        //   if (readingISASoC > 0) SOC_Bat_Act=readingISASoC;
        //   else SOC_Bat_Act = 0;
        //   break;
        // }
      }
    ISA_watchdog=millis();
}
void Check_Elcon(CAN_message_t incoming)
{
  switch (incoming.id)
      {
        case canID_ElconChargerFback:
          {    
            long readingElconChargerVolt=0;
            long readingElconChargerAmp=0;
            long readingElconChargerTemp=0;
            uint8_t readingElconChargerStatus=0;

            readingElconChargerVolt = (long)((incoming.buf[0] << 8) | (incoming.buf[1]));
            // readingElconChargerVolt = readingElconChargerVolt/10; 
            if (readingElconChargerVolt > 0) ElconCharger_VoltOutput=readingElconChargerVolt;
            else ElconCharger_VoltOutput = 0;

            readingElconChargerAmp = (long)((incoming.buf[2] << 8) | (incoming.buf[3]));
            // readingElconChargerAmp = readingElconChargerAmp/10; 
            if (readingElconChargerAmp > 0) ElconCharger_AmpOutput=readingElconChargerAmp;
            else ElconCharger_AmpOutput = 0;

            readingElconChargerStatus = (uint8_t)(incoming.buf[4]);
            if (readingElconChargerStatus >= 0) 
              {
                ElconCharger_msgInvalid=false;
                ElconCharger_HardwareError=(readingElconChargerStatus>>0)&0x01;
                ElconCharger_TempError=(readingElconChargerStatus>>1)&0x01;
                ElconCharger_InVoltError=(readingElconChargerStatus>>2)&0x01;
                ElconCharger_BatVoltError=(readingElconChargerStatus>>3)&0x01;
                ElconCharger_CommTimeout=(readingElconChargerStatus>>4)&0x01;
              }
            else ElconCharger_msgInvalid = true;

            readingElconChargerTemp = (long)(incoming.buf[5]);
            // readingElconChargerTemp = readingElconChargerAmp/10; 
            readingElconChargerTemp=readingElconChargerTemp-40;//40C offset; 0 from charger = -40C; operating range -40...+85C; >85C = overtemp; 90C=shutdown
            ElconCharger_Temp=readingElconChargerTemp;

            Can1.write(incoming); //copy-paste Elcon Charger feedback from can2 to can1 for display
            // send charger ctrl data
            ElconCharger_CtrlReq=true;
          }
        case canID_ElconDCDCFback:
          {    
            long readingElconDCDCVolt=0;
            long readingElconDCDCAmp=0;
            long readingElconDCDCTemp=0;
            uint16_t readingElconDCDCStatus=0;

            readingElconDCDCVolt = (long)((incoming.buf[0] << 8) | (incoming.buf[1]));
            // readingElconDCDCVolt = readingElconDCDCVolt/10; 
            if (readingElconDCDCVolt > 0) ElconDCDC_VoltOutput=readingElconDCDCVolt;
            else ElconDCDC_VoltOutput = 0;

            readingElconDCDCAmp = (long)((incoming.buf[2] << 8) | (incoming.buf[3]));
            // readingElconDCDCAmp = readingElconDCDCAmp/10; 
            if (readingElconDCDCAmp > 0) ElconDCDC_AmpOutput=readingElconDCDCAmp;
            else ElconDCDC_AmpOutput = 0;

            readingElconDCDCStatus = (uint16_t)((incoming.buf[4] << 8) | (incoming.buf[5]));
            if (readingElconDCDCStatus >= 0) 
            {
              ElconDCDC_msgInvalid=false;
              ElconDCDC_HVILError=(readingElconDCDCStatus>>15)&0x01; //0:Lock accomplish,1Non-Lock
              ElconDCDC_WaterFanSts=(readingElconDCDCStatus>>14)&0x01; //0:FAN OFF。1: FAN ON
              ElconDCDC_StopError=(readingElconDCDCStatus>>13)&0x01; //1:Error, 0:No error
              ElconDCDC_WaterFanSts2=(readingElconDCDCStatus>>12)&0x01; //0:off。1:ON
              ElconDCDC_CommTimeout=(readingElconDCDCStatus>>11)&0x01; //1:Error, 0:No error
              ElconDCDC_HardwareError=(readingElconDCDCStatus>>10)&0x01; //1:Error, 0:No error
              ElconDCDC_Sts=(readingElconDCDCStatus>>9)&0x1;//1:working, 0:stopped
              ElconDCDC_Ready=(readingElconDCDCStatus>>8)&0x1; //initialisation 0:uncompleted; 1:completed
              ElconDCDC_OutOverAmp=(readingElconDCDCStatus>>6)&0x1; //1:Error, 0:No error
              ElconDCDC_OutUnderVolt=(readingElconDCDCStatus>>5)&0x1; //1:Error, 0:No error
              ElconDCDC_OutOverVolt=(readingElconDCDCStatus>>4)&0x1; //1:Error, 0:No error
              ElconDCDC_InUnderVolt=(readingElconDCDCStatus>>3)&0x1; //1:Error, 0:No error
              ElconDCDC_InOverVolt=(readingElconDCDCStatus>>2)&0x1; //1:Error, 0:No error
              ElconDCDC_OverTemp=(readingElconDCDCStatus>>1)&0x1; //1:Error, 0:No error
              ElconDCDC_HighTemp=(readingElconDCDCStatus)&0x1; ////1:High temp, 0:Normal temp
            }
            else ElconDCDC_msgInvalid = true;

            readingElconDCDCTemp = (long)(incoming.buf[7]);
            // readingElconDCDCTemp=readingElconDCDCTemp-32;
            // readingElconDCDCTemp=readingElconDCDCTemp*5;
            // readingElconDCDCTemp=readingElconDCDCTemp/9;
            readingElconDCDCTemp=readingElconDCDCTemp-60; //60C offset; 0 from dcdc = -60C; operating range -40...+60C
            ElconDCDC_Temp=readingElconDCDCTemp;
            Can1.write(incoming); //copy-paste Elcon DCDC feedback from can2 to can1
            break;
          }
      }
    Elcon_watchdog=millis();
}
void Check_LIM(CAN_message_t incoming)
{
  //Serial.println(incoming.id);
  switch (incoming.id)
    {
      case 0x337:    
        LIM_Stat_Hook_Pos = ((incoming.buf[0] >> 0) & 3);
        LIM_Stat_Hook_Loc = ((incoming.buf[0] >> 2) & 3);
      break;
      case 0x390:    
        LIM_Stat_Hospitality = ((incoming.buf[0] >> 0) & 15);
      break;
      case 0x29E:    
        LIM_DCSE_Prot_Num = incoming.buf[7];
        LIM_DCSE_V_Trhd = (uint16_t)((incoming.buf[6] << 8) | (incoming.buf[5]));
        LIM_DCSE_I_Avbl = (uint16_t)((incoming.buf[4] << 8) | (incoming.buf[3]));
        LIM_DCSE_V_Avbl = (uint16_t)((incoming.buf[2] << 8) | (incoming.buf[1]));
        LIM_DCSE_Stat_Iso = ((incoming.buf[0] >> 6) & 3);
        LIM_DCSE_Stat_Chg = ((incoming.buf[0] >> 2) & 15);
        LIM_DCSE_Weld_Det = ((incoming.buf[0] >> 0) & 3);
      break;
      case 0x2B2:    
        LIM_DCSE_Rst_Tme_Chg = (uint16_t)((incoming.buf[7] << 8) | (incoming.buf[6]));
        LIM_DCSE_V_LIM_Reached = ((incoming.buf[5] >> 6) & 3);
        LIM_DCSE_I_LIM_Reached = ((incoming.buf[5] >> 4) & 3);
        LIM_DCSE_DC_Stp_Crt= ((incoming.buf[5] >> 2) & 3);
        LIM_DCSE_Bat_Prob= ((incoming.buf[5] >> 0) & 3);
        LIM_DCSE_Bat_Comp= ((incoming.buf[4] >> 6) & 3);
        LIM_DCSE_Loc_Stat= ((incoming.buf[4] >> 4) & 3);
        LIM_DCSE_Chg_Prob= ((incoming.buf[4] >> 2) & 3);
        LIM_DCSE_Chg_Stat= ((incoming.buf[4] >> 0) & 3);
        LIM_DCSE_I_Current = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
        LIM_DCSE_V_Current = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
      break;
      case 0x2EF:    
        LIM_DCSE_Eng_Trsm = incoming.buf[7];
        LIM_DCSE_Location = ((incoming.buf[6] >> 6) & 3);
        LIM_DCSE_P_LIM_Reached = ((incoming.buf[6] >> 4) & 3);
        LIM_DCSE_I_Tol = (uint16_t)((((incoming.buf[6] >> 0) & 15) << 8) | (incoming.buf[5]));
        LIM_DCSE_I_Riple =    (uint16_t)((incoming.buf[4] << 4) | ((incoming.buf[3] >> 4) & 4));
        LIM_DCSE_I_Min_Avbl = (uint16_t)((((incoming.buf[3] >> 0) & 15) << 8) | (incoming.buf[2]));
        LIM_DCSE_V_Min_Avbl = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
      break;
      case 0x3B4:    
        LIM_V_CP_DC = incoming.buf[7]*2;
        LIM_Charger_Type = incoming.buf[6];
        LIM_Stat_Line_Held = ((incoming.buf[4] >> 5) & 3);
        LIM_Stat_Line_Chademo = ((incoming.buf[4] >> 3) & 3); 
        LIM_Stat_Pilot = ((incoming.buf[4] >> 0) & 7);
        LIM_Stat_Crt_Chademo = ((incoming.buf[3] >> 4) & 15); 
        LIM_Stat_Loc_CP = ((incoming.buf[3] >> 2) & 3);
        LIM_Stat_Flap_Lock = ((incoming.buf[3] >> 0) & 3);
        LIM_Chg_Enable = ((incoming.buf[2] >> 6) & 3);
        LIM_Chg_End_Req = ((incoming.buf[2] >> 4) & 3);
        LIM_Stat_Unloc_But= ((incoming.buf[2] >> 2) & 3);
        LIM_Stat_Line_Plg = ((incoming.buf[2] >> 0) & 3);
        LIM_ACSE_I_Avbl_Line = incoming.buf[1]; 
        LIM_ACSE_I_Avbl_Grid = incoming.buf[0];          
      break;
      case 0x272:    
        LIM_Chg_AC_Avbl = ((incoming.buf[3] >> 0) & 7);
        LIM_Chg_DC_Avbl = ((incoming.buf[3] >> 3) & 7);;
        LIM_Stat_DC_Rel = ((incoming.buf[2] >> 2) & 0x3F);
        LIM_Stat_Pos_Flap = ((incoming.buf[2] >> 0) & 3);
        LIM_Alive = ((incoming.buf[1] >> 0) & 15);
      break;
    }
  LIM_watchdog=millis();
}
void Check_BMS(CAN_message_t incoming)
{
  switch (incoming.id)
    {
      case canID_BMSInfo:
        {    
        uint8_t readingBMSInfo=0;
        uint16_t readingBMSCap=0;
        int readingBMSCellsBal=0xFF;

        readingBMSCap=(uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
        if (readingBMSCap > 0) BMS_CapacityAh=readingBMSCap;
        else BMS_CapacityAh = 0;

        readingBMSInfo = incoming.buf[4]; 
        if (readingBMSInfo > 0) BMS_State=readingBMSInfo;
        else BMS_State = 0;

        readingBMSCellsBal = incoming.buf[5]; 
        BMS_CellsBal=readingBMSCellsBal;
        
        break;
        }
      case canID_BMSLimits:
        {    
        uint16_t readingBMS_ChargeVoltLim = 0; 
        uint16_t readingBMS_ChargeAmpLim = 0; 
        uint16_t readingBMS_disChargeVoltLim = 0; 
        uint16_t readingBMS_disChargeAmpLim = 0; 

        readingBMS_ChargeVoltLim = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
        // readingBMS_ChargeVoltLim = readingBMS_ChargeVoltLim/10; 
        if (readingBMS_ChargeVoltLim > 0) BMS_ChargeVoltLim=readingBMS_ChargeVoltLim;
        else BMS_ChargeVoltLim = 0;
        BMS_CapacityWhCharge=(uint32_t) (BMS_CapacityAh*BMS_ChargeVoltLim);

        readingBMS_ChargeAmpLim = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
        // readingBMS_ChargeAmpLim = readingBMS_ChargeAmpLim/10; 
        if (readingBMS_ChargeAmpLim > 0) BMS_ChargeAmpLim=readingBMS_ChargeAmpLim;
        else BMS_ChargeAmpLim = 0;

        readingBMS_disChargeVoltLim = (uint16_t)((incoming.buf[7] << 8) | (incoming.buf[6]));
        // readingBMS_disChargeVoltLim = readingBMS_disChargeVoltLim/10; 
        if (readingBMS_disChargeVoltLim > 0) BMS_disChargeVoltLim=readingBMS_disChargeVoltLim;
        else BMS_disChargeVoltLim = 0;

        readingBMS_disChargeAmpLim = (uint16_t)((incoming.buf[5] << 8) | (incoming.buf[4]));
        // readingBMS_disChargeAmpLim = readingBMS_disChargeAmpLim/10; 
        if (readingBMS_disChargeAmpLim > 0) BMS_disChargeAmpLim=readingBMS_disChargeAmpLim;
        else BMS_disChargeAmpLim = 0;

        break;
        }

      case canID_BMSSOC:
        {    
        uint16_t readingBMS_SOC = 0; 

        readingBMS_SOC = (uint16_t)((incoming.buf[5] << 8) | (incoming.buf[4]));
        if (readingBMS_SOC > 0) BMS_SOC=readingBMS_SOC;
        else BMS_SOC = 0;

        break;
        }
      case canID_BMSLowHigh:
        {    
        uint16_t readingBMS_MaxCellsVolt = 0; 
        uint16_t readingBMS_MinCellsVolt = 0; 
        uint16_t readingBMS_MaxCellsTemp = 0; 
        uint16_t readingBMS_MinCellsTemp = 0; 

        readingBMS_MinCellsVolt = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
        // readingBMS_MinCellsVolt = readingBMS_MinCellsVolt/1000;
        if (readingBMS_MinCellsVolt > 0) BMS_CellsVoltMin=readingBMS_MinCellsVolt*1.0f;
        else BMS_CellsVoltMin = 0;

        readingBMS_MaxCellsVolt = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
        // readingBMS_MaxCellsVolt = readingBMS_MaxCellsVolt/1000;
        if (readingBMS_MaxCellsVolt > 0) BMS_CellsVoltMax=readingBMS_MaxCellsVolt*1.0f;
        else BMS_CellsVoltMax = 0;

        readingBMS_MinCellsTemp = (uint16_t) incoming.buf[5] << 8 | incoming.buf[4];
        // readingBMS_MinCellsTemp = readingBMS_MinCellsTemp-273.15;
        if (readingBMS_MinCellsTemp > 0) BMS_CellsTempMin=readingBMS_MinCellsTemp;
        else BMS_CellsTempMin = 0;

        readingBMS_MaxCellsTemp = (uint16_t) incoming.buf[7] << 8 | incoming.buf[6];
        // readingBMS_MaxCellsTemp = readingBMS_MaxCellsTemp-273.15;
        if (readingBMS_MaxCellsTemp > 0) BMS_CellsTempMax=readingBMS_MaxCellsTemp;
        else BMS_CellsTempMax = 0;

        break;
        }
      case canID_BMSStatus:
        {    
          uint16_t readingBMS_PackVoltage = 0; 
          uint16_t readingBMS_Current = 0; 
          int16_t readingBMS_AvgTemp = 0; 

          readingBMS_PackVoltage = (uint16_t)((incoming.buf[1] << 8) | (incoming.buf[0]));
          if (readingBMS_PackVoltage > 0) BMS_PackVolt=readingBMS_PackVoltage;
          else BMS_PackVolt = 0;

          readingBMS_Current = (uint16_t)((incoming.buf[3] << 8) | (incoming.buf[2]));
          // readingBMS_Current=readingBMS_Current/100; //miliamps to amps in 0.1A steps
          // readingBMS_Current=8192-readingBMS_Current;
          // BMS_BatAmp=(long)(readingBMS_Current*(-1));
          // if (readingBMS_Current > 0x7ffffff) BMS_BatAmp=-BMS_BatAmp;
          BMS_BatAmp = (int16_t)readingBMS_Current; //return back to signed value, note this value is in amps in 0.1A steps

          readingBMS_AvgTemp = (int16_t)((incoming.buf[5] << 8) | (incoming.buf[4]));
          BMS_AvgTemp=readingBMS_AvgTemp;

          break;
        }
      case canID_BMSAlarmsWarnings:
        {
          BMS_Error_CellOverVLimit = (uint8_t) ((incoming.buf[0]>>2)&0x01); //alarm: cell over high Voltage setpoint
          BMS_Error_CellUnderVLimit = (uint8_t) ((incoming.buf[0]>>4)&0x01); //alarm: cell under low Voltage setpoint
          BMS_Error_CellOverTLimit = (uint8_t) ((incoming.buf[0]>>6)&0x01); //alarm: cell over high Temperature setpoint
          BMS_Error_CellUnderVLimit = (uint8_t) (incoming.buf[1]&0x01); //alarm: cell under low Temperature setpoint
          BMS_Error_CellImbalance = (uint8_t) (incoming.buf[3]&0x01); //alarm: cell voltage imbalance
          BMS_Warn_CellOverVLimit = (uint8_t) ((incoming.buf[4]>>2)&0x01); //warning: cell over high Voltage setpoint
          BMS_Warn_CellUnderVLimit = (uint8_t) ((incoming.buf[4]>>4)&0x01); //warning: cell under low Voltage setpoint
          BMS_Warn_CellOverTLimit = (uint8_t) ((incoming.buf[4]>>6)&0x01); //warning: cell over high Temperature setpoint
          BMS_Warn_CellUnderTLimit = (uint8_t) (incoming.buf[5]&0x01); //warning: cell under low Temperature setpoint
          break;
        }
    }
  BMS_watchdog=millis();
}
void Check_Drive(CAN_message_t incoming)
{
  switch (incoming.id)
      {
        case canID_Drive_Stat:
        {
          int8_t readingDriveDir=Drive_Invalid;
          uint8_t readingDrive_OpMode=Drive_OPMODE_Invalid;
          uint16_t readingDrive_Status=Drive_Stat_Invalid;
          Drive_MotorTemp=0xFFFF;
          Drive_HtSnkTemp=0xFFFF;
          
          readingDrive_OpMode = (uint8_t)(incoming.buf[0]);
          if (readingDrive_OpMode<=6) Drive_OpMode=readingDrive_OpMode;
          else Drive_OpMode=Drive_OPMODE_Invalid;

          readingDrive_Status = (uint16_t)((incoming.buf[2] << 8) | (incoming.buf[1]));
          if ((readingDrive_Status>>10)==0) Drive_Status=readingDrive_Status;
          else Drive_Status=Drive_Stat_Invalid;

          readingDriveDir = (int8_t)(incoming.buf[3]);
          if (readingDriveDir==Drive_N ||readingDriveDir==Drive_F || readingDriveDir==Drive_R) Drive_Dir=readingDriveDir;
          else Drive_Dir=Drive_Invalid;

          Drive_HtSnkTemp=(uint16_t)((incoming.buf[5] << 8) | (incoming.buf[4])); // scale 1/100
          Drive_MotorTemp=(uint16_t)((incoming.buf[7] << 8) | (incoming.buf[6])); // scale 1/100

          break;
        }
      }
    Drive_watchdog=millis();
}

void send_Drive_Cmd(void)   //send Drive Command
{
  CAN_message_t outgoing;
  uint32_t Drive_Cmd_temp_data[2];
  uint32_t Drive_Cmd_CRC = 0xffffffff;

  Drive_Cmd_Cruise=false;
  Drive_Cmd_Brake=false;
  Drive_Cmd_Fwd=false;
  Drive_Cmd_Rev=false;
  Drive_Cmd_BMS=false;
  Drive_Cmd_Pot1=0;
  Drive_Cmd_Pot2=0;
  Drive_Cmd_CruiseSpeed=0;

  //turn on regen when batteries aren't close to full and temps are reasonable
  if (BMS_SOC<=850 && (BMS_CellsTempMax-273)<38 && (BMS_CellsTempMin-273)>=12) {Drive_Cmd_RegenPreset=100;} //apply fully defined regen
  else {Drive_Cmd_RegenPreset=0;}
  // Drive_Cmd_RegenPreset=100;
  
  Drive_Cmd_Ctr+=1;

  uint8_t cruise = ((uint8_t) Drive_Cmd_Cruise)&0x1;
  uint8_t start = ((uint8_t) Drive_Cmd_Start)&0x1;
  uint8_t brake = ((uint8_t) Drive_Cmd_Brake)&0x1;
  uint8_t fwd = ((uint8_t) Drive_Cmd_Fwd)&0x1;
  uint8_t rev = ((uint8_t) Drive_Cmd_Rev)&0x1;
  uint8_t bms = ((uint8_t) Drive_Cmd_BMS)&0x1;
  
  uint16_t Pot = Drive_Cmd_Pot1 & 0xFFF;
  uint16_t Pot2 = Drive_Cmd_Pot2 & 0xFFF;
  // uint8_t canio = ( Drive_Cmd_Cruise | Drive_Cmd_Start<<1|Drive_Cmd_Brake<<2|Drive_Cmd_Fwd<<3|Drive_Cmd_Rev<<4|Drive_Cmd_BMS<<5 ) & 0x3F;
  uint8_t canio = (cruise | (start<<1) | (brake<<2) | (fwd<<3) | (rev<<4) | (bms<<5)) & 0x3F;
  uint8_t ctr = Drive_Cmd_Ctr & 0x3;
  uint16_t cruise_speed = Drive_Cmd_CruiseSpeed & 0x3FFF;
  uint8_t regen_preset = Drive_Cmd_RegenPreset & 0x7F;

  Drive_Cmd_temp_data[0] = Pot | (Pot2 << 12) | (canio << 24) | (ctr << 30);
  Drive_Cmd_temp_data[1] = cruise_speed | (ctr << 14) | (regen_preset << 16);
  Drive_Cmd_CRC = crc32_word(Drive_Cmd_CRC, Drive_Cmd_temp_data[0]);
  Drive_Cmd_CRC = crc32_word(Drive_Cmd_CRC, Drive_Cmd_temp_data[1]);

  outgoing.id = canID_Drive_Cmd;
  outgoing.flags.extended = false;
  outgoing.len = 8;    
  outgoing.buf[0]= (Drive_Cmd_temp_data[0]) & 0xFF;
  outgoing.buf[1]= (Drive_Cmd_temp_data[0]>>8) & 0xFF;
  outgoing.buf[2]= (Drive_Cmd_temp_data[0]>>16) & 0xFF;
  outgoing.buf[3]= (Drive_Cmd_temp_data[0])>>24 & 0xFF;
  outgoing.buf[4]= (Drive_Cmd_temp_data[1]) & 0xFF;
  outgoing.buf[5]= (Drive_Cmd_temp_data[1]>>8) & 0xFF;
  outgoing.buf[6]= (Drive_Cmd_temp_data[1]>>16) & 0xFF;
  // outgoing.buf[7]= (Drive_Cmd_temp_data[1])>>24 & 0xFF;
  outgoing.buf[7]=  Drive_Cmd_CRC & 0xFF;
  Can1.write(outgoing);
}

static uint32_t crc32_word(uint32_t Crc, uint32_t Data)
{
  uint8_t i;

  Crc ^= Data;

  for(i=0; i<32; i++)
    if (Crc & 0x80000000)
      Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      Crc = (Crc << 1);

  return(Crc);
}

void Check_IBooster(CAN_message_t incoming)
{
  switch (incoming.id)
      {
        case canID_stsIBooster:
        {
          uint8_t readingIBST_driverBrakeApply=0xff;
          uint8_t readingIBST_iBoosterStatus=0xff;
          uint8_t readingIBST_internalState=0xff;
          uint16_t readingIBST_RodLen=0xffff;
          
          readingIBST_driverBrakeApply = (uint8_t)(incoming.buf[2] & 0x03);
          if (readingIBST_driverBrakeApply==BRAKES_NOT_APPLIED || readingIBST_driverBrakeApply==DRIVER_APPLYING_BRAKES || readingIBST_driverBrakeApply==FAULT || readingIBST_driverBrakeApply==NOT_INIT_OR_OFF) 
            {
              IBST_driverBrakeApply=readingIBST_driverBrakeApply;
            }
          else 
            {
              IBST_driverBrakeApply=IBST_Invalid;
            }

          readingIBST_iBoosterStatus = (uint8_t)((incoming.buf[1] >> 4) & 0x07);
          if (readingIBST_iBoosterStatus==IBOOSTER_ACTIVE_GOOD_CHECK || readingIBST_iBoosterStatus==IBOOSTER_ACTUATION || readingIBST_iBoosterStatus==IBOOSTER_DIAGNOSTIC || readingIBST_iBoosterStatus==IBOOSTER_FAILURE ||readingIBST_iBoosterStatus==IBOOSTER_INIT || readingIBST_iBoosterStatus==IBOOSTER_OFF || readingIBST_iBoosterStatus==IBOOSTER_READY) 
            {
              IBST_iBoosterStatus=readingIBST_iBoosterStatus;
            }
          else 
            {
              IBST_iBoosterStatus=IBST_Invalid;
            }
          readingIBST_internalState = (uint8_t)((incoming.buf[2] >> 2) & 0x07);
          if (readingIBST_internalState==DIAGNOSTIC || readingIBST_internalState==EXTERNAL_BRAKE_REQUEST || readingIBST_internalState==LOCAL_BRAKE_REQUEST || readingIBST_internalState==NO_MODE_ACTIVE || readingIBST_internalState==POST_DRIVE_CHECK || readingIBST_internalState==PRE_DRIVE_CHECK || readingIBST_internalState==TRANSITION_TO_IDLE) 
            {
              IBST_internalState=readingIBST_internalState;
            }
          else 
            {
              IBST_internalState=IBST_Invalid;
            }
          readingIBST_RodLen = (uint16_t)((((incoming.buf[4]) & 0x01)<<11) | ((incoming.buf[3])<<3) | ((incoming.buf[2] >> 5) & 0x07));
          IBST_rodLen = readingIBST_RodLen;
          break;
        }
      }
    IBooster_watchdog=millis();
}

// Messages needed to be sent to periodically
void periodicMSG(void)
{
  timer_10ms_send.end(); //pause interrupt feature
  // 10ms
  send_112h();
  //questionable messages. maybe needed!? tbd
  // send_512h();
  // send_560h();
  // send_330h();
  // send_397h();

  // 20ms
  ctr_20ms_LIMsend++;
  if(ctr_20ms_LIMsend>=2) 
    {
      ctr_20ms_LIMsend=0; 
      send_1A1h();
    }
      
  // 200ms
  ctr_200ms_LIMsend++;
  if(ctr_200ms_LIMsend>=20) 
    {
      ctr_200ms_LIMsend=0; 
      send_2FAh();
      send_432h();
      send_51Ah();
      send_540h();
      send_510h();

      // 1s
      ctr_1s_LIMsend++;
      if(ctr_1s_LIMsend>=5) 
        {
          ctr_1s_LIMsend=0; 
          send_328h();
          send_3E8h();
          send_3F9h();

          // 4s
          ctr_4s_LIMsend++;
          if(ctr_4s_LIMsend>=4) 
            {
              ctr_4s_LIMsend=0;
              send_2FCh();
              send_2A0h();
              send_3A0h();
            }
        }
    }

  // 100ms
  ctr_100ms_LIMsend++;
  if(ctr_100ms_LIMsend>=10) 
    {
      ctr_100ms_LIMsend=0;
      send_03Ch();
      send_3E9h();
      send_431h();
      send_12Fh();
      send_2F1h();
      // send_ctrlDCDC(); //DCDC command
    }

  //100ms
  ctr_100ms_DRIVEsend++;
  if(ctr_100ms_DRIVEsend>=10) 
    {
      ctr_100ms_DRIVEsend=0; 
      send_Drive_Cmd();
    }  


  timer_10ms_send.begin(periodicMSG, 10000); //restart interrupt feature
}

// every 10ms
void send_112h(void)   //Current battery voltage up to 819V HV opening request status, DC link voltage from the drive
{
  uint16_t temp;
  temp = ISA_MainPOSVolt/40;
  CAN_message_t outgoing;
  outgoing.id = 0x112;
  outgoing.flags.extended = false;
  outgoing.len = 8;    
  outgoing.buf[0] = ISA_BatAmp & 0x00FF;  //Battery current LSB. Scale 0.1 offset 819.2. 16 bit unsigned int
  outgoing.buf[1] = ISA_BatAmp >> 8;     //Battery current MSB. Scale 0.1 offset 819.2. 16 bit unsigned int
  outgoing.buf[2] = ISA_BatVolt & 0x00FF;  //Battery voltage LSB. Scale 0.1 offset 819.2. 16 bit unsigned int
  outgoing.buf[3] = ISA_BatVolt >> 8;     //Battery voltage MSB. Scale 0.1 offset 819.2. 16 bit unsigned int
  outgoing.buf[4] = BMS_SOC & 0x00FF;  //Battery SOC LSB. 12 bit unsigned int. Scale 0.1. 0-100%
  outgoing.buf[5] = BMS_SOC >> 8;     //Battery SOC MSB. 12 bit unsigned int. Scale 0.1. 0-100%
  // outgoing.buf[6] = (byte)((0xF) | (Bat_Rel_Opn_Req_Fast << 2) | (Bat_Rel_Opn_Req)) ; // Request to open isolating relay (quickly) 0=no statement, 1=not active, 2=active, 3=invalid
  outgoing.buf[6] = 0x65; ////Low nibble battery status. Seem to need to be 0x65.
  outgoing.buf[7] = temp;         // Battery voltage. Scale 4. 8 bit unsigned int. 
  Can1.write(outgoing);
}
void send_1A1h(void)   //Driving speed + assessment
{
  CAN_message_t outgoing;
  outgoing.id = 0x1A1;
  outgoing.flags.extended = false;
  outgoing.len = 5;  
  outgoing.buf[0] = 0x7C; 
  outgoing.buf[1] = 0xCB;  
  //vehicle speed
  if (BMS_State==BMS_Drive)
    {
      outgoing.buf[2] = 0x80;   //vehicle speed ~5mph LSB
      outgoing.buf[3] = 0x02;   //vehicle speed ~5mph MSB 
    }
  else
    {
      outgoing.buf[2] = 0x00;   //vehicle speed 0mph LSB
      outgoing.buf[3] = 0x00;   //vehicle speed 0mph MSB          
    }   
  outgoing.buf[4] = 0x8A;
  Can1.write(outgoing); 
}
// every 100ms
void send_03Ch(void)   //vehicle status msg
{
  CAN_message_t outgoing;
  outgoing.id = 0x03c;
  outgoing.flags.extended = false;
  outgoing.len = 8;  
  outgoing.buf[0] = 0xff;//vehicle status msg
  outgoing.buf[1] = 0x5f;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0x00;
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0xff;
  outgoing.buf[7] = 0xff;
  Can1.write(outgoing); 
}
void send_3E9h(void)   //Desired (up to!!!!!) 65534 Wh, charging status, readiness to charge, target current DC up to 252A, contactor, remaining charging time
{
  uint32_t temp_BMS_CapacityWhCharge=0;
  temp_BMS_CapacityWhCharge=BMS_CapacityWhCharge/10;
  ChargePower=(ChargePower & 0xFFF);
  CAN_message_t outgoing;
  outgoing.id = 0x3E9;
  outgoing.flags.extended = false;
  outgoing.len = 8;  

  //16-bit battery capacity only fits 
  if (temp_BMS_CapacityWhCharge<=0xffff)
    {
      outgoing.buf[0] =  temp_BMS_CapacityWhCharge & 0xff;                // Battery capacity LSB in Wh
      outgoing.buf[1] =  temp_BMS_CapacityWhCharge >> 8;                // Battery capacity MSB
    }
  else
    {
      outgoing.buf[0] =  0xfe;                // Battery capacity LSB in Wh
      outgoing.buf[1] =  0xff;                // Battery capacity MSB
    }
  // outgoing.buf[2] = (0xF |(ChargeStatus<<4));         //charge status in bits 4-7.goes to 1 then 2.8 secs later to 2. Plug locking???. Charge request in lower nibble. 1 when charging. 0 when not charging.
  outgoing.buf[2] = (((uint8_t) ChargeStatus<<4)|((uint8_t) ChargeReq));
  outgoing.buf[3] = (((ChargePower)<<4)|(uint8_t)ChargeReady);          // Ready to charge
  outgoing.buf[4] = ChargePower>>4;                           // invalid 
  outgoing.buf[5] = I_Target_DCCharge & 0xff;   //LSB of the DC ccs current command
  outgoing.buf[6] = ((CCS_Contactor_Ctrl<<4)|(I_Target_DCCharge>>12));   //bits 0 and 1 MSB of the DC ccs current command.Upper nibble is DC ccs contactor control. Observed in DC fc logs only.
    //transitions from 0 to 2 and start of charge but 2 to 1 to 0 at end. Status and Ready operate the same as in AC logs.
  outgoing.buf[7] = ctr_mins_LIMsend_EOC;    // end of charge timer.
  Can1.write(outgoing);
}
void send_431h(void) //LIM needs this, but doesn't do anything
{
  CAN_message_t outgoing;
  outgoing.id = 0x431;
  outgoing.flags.extended = false;
  outgoing.len = 8;                
  outgoing.buf[0] = 0xca;
  outgoing.buf[1] = 0xff;
  outgoing.buf[2] = 0x0b;
  outgoing.buf[3] = 0x02;
  outgoing.buf[4] = 0x69;
  outgoing.buf[5] = 0x26;
  outgoing.buf[6] = 0xf3;
  outgoing.buf[7] = 0x4b;
  Can1.write(outgoing);
}
void send_12Fh(void) //wake up message
{
  CAN_message_t outgoing;
  outgoing.id=0x12f;
  outgoing.flags.extended = false;
  outgoing.len = 8;                
  outgoing.buf[0] = 0xf5;//Wake up message.
  outgoing.buf[1] = 0x28;
  // if(Drive_Dir!=Drive_Invalid) outgoing.buf[2] = 0x8a;//ignition on
  // else outgoing.buf[2] = 0x86;//ignition off
  outgoing.buf[2] = Ignition_Status_Sim;
  outgoing.buf[3] = 0x1d;
  outgoing.buf[4] = 0xf1;
  outgoing.buf[5] = 0x35;
  outgoing.buf[6] = 0x30;
  outgoing.buf[7] = 0x80;
  Can1.write(outgoing);
}
void send_2F1h(void) //Lim command 2. Used in DC mode
{
  CAN_message_t outgoing;
  uint16_t V_limit=0;
  if(LIM_SM==4 || LIM_SM==5) V_limit=ISA_BatVolt;// drop vlim only during precharge
  else V_limit=415*10;//set to 415v in all other states
  uint8_t I_limit=125;//125A limit. may not work
  uint16_t temp_ctr_secs_LIMsend_FullSOC=ctr_secs_LIMsend_FullSOC/10;
  uint16_t temp_ctr_secs_LIMsend_BulkSOC=ctr_secs_LIMsend_BulkSOC/10;

  outgoing.id=0x2f1;
  outgoing.flags.extended = false;
  outgoing.len = 8;                
  outgoing.buf[0] = V_limit & 0xFF;  //Charge voltage limit LSB. 14 bit signed int.scale 0.1 0xfa2=4002*.1=400.2Volts
  outgoing.buf[1] = V_limit >> 8;  //Charge voltage limit MSB. 14 bit signed int.scale 0.1
  outgoing.buf[2] = I_limit;  //Fast charge current limit. Not used in logs from 2014-15 vehicle so far. 8 bit unsigned int. scale 1.so max 254amps in theory...
  outgoing.buf[3] = temp_ctr_secs_LIMsend_FullSOC & 0xFF;  //time remaining in seconds to hit soc target from byte 7 in AC mode. LSB. 16 bit unsigned int. scale 10.Full SOC.
  outgoing.buf[4] = temp_ctr_secs_LIMsend_FullSOC >> 8;  //time remaining in seconds to hit soc target from byte 7 in AC mode. MSB. 16 bit unsigned int. scale 10.Full SOC.
  outgoing.buf[5] = temp_ctr_secs_LIMsend_BulkSOC & 0xFF;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. LSB. 16 bit unsigned int. scale 10.Bulk SOC.
  outgoing.buf[6] = temp_ctr_secs_LIMsend_BulkSOC >> 8;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. MSB. 16 bit unsigned int. scale 10.Bulk SOC.
  outgoing.buf[7] = 0xA0;  //Fast charge SOC target. 8 bit unsigned int. scale 0.5. 0xA0=160*0.5=80%
  Can1.write(outgoing);
}

// every 200ms
void send_2FAh(void)    //DC charging end, AC charging end, for SAE-J2847, should charging status
{
  CAN_message_t outgoing;
  outgoing.id = 0x2FA;
  outgoing.flags.extended = false;
  outgoing.len = 8;  
  //Lim command 3. Used in DC mode.
  if(LIM_Stat_Pilot==0x4||LIM_Stat_Pilot==0x5) outgoing.buf[0] = 0xFC; //FD at standby, change to FC on 5% pilot. Change back to FD during energy transfer
  else outgoing.buf[0] = 0xFD; 
  outgoing.buf[1] = 0xFF;//these bytes are used as a timer during energy transfer but not at setup
  // outgoing.buf[2] = (uint8_t)DC_Charge_Phase<<4;
  outgoing.buf[2] = (uint8_t)((DC_Charge_Phase<<4) | (ChargeEnd << 2) | (DCChargeEnd));       // End of charging and charging phases are specified at d. to 9 instead of 4
  outgoing.buf[3] = 0xFF;       // invalid  
  outgoing.buf[4] = 0xFF;
  outgoing.buf[5] = 0xFF;
  outgoing.buf[6] = 0xFF;
  outgoing.buf[7] = 0xFF;
  Can1.write(outgoing);
}
void send_432h(void)   //Charge level 0-100% from the BMS for the display
{
  CAN_message_t outgoing;
  uint16_t scaledSOC=BMS_SOC/5; //BMS_SOC is 0...100% in 0,1 steps, converting to 0.5 steps
  outgoing.id = 0x432;
  outgoing.flags.extended = false;
  outgoing.len = 8;   
  outgoing.buf[0] = 0x2c;//BMS soc msg. May need to be dynamic
  outgoing.buf[1] = 0xe2;
  outgoing.buf[2] = 0x10;
  outgoing.buf[3] = 0xa3;
  outgoing.buf[4] = scaledSOC;           //Current charge level 0..100% in 0.5% steps
  outgoing.buf[5] = 0xff;
  outgoing.buf[6] = 0x02;
  outgoing.buf[7] = 0xff;
  Can1.write(outgoing);
}
void send_51Ah(void)   //Sleep preventer / network management
{
  CAN_message_t outgoing;
  outgoing.id = 0x51A;
  outgoing.flags.extended = false;
  outgoing.len = 8;   
  outgoing.buf[0] = 0x00;
  outgoing.buf[1] = 0x00;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0x50;
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0x00;
  outgoing.buf[7] = 0x1a; //try 0x2a?
  Can1.write(outgoing);
}
void send_540h(void)   //Sleep preventer / network management 2
{
  CAN_message_t outgoing;
  outgoing.id = 0x540;
  outgoing.flags.extended = false;
  outgoing.len = 8;   
  outgoing.buf[0] = 0x00;
  outgoing.buf[1] = 0x00;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0xfd;
  outgoing.buf[5] = 0x3c;
  outgoing.buf[6] = 0xff;
  outgoing.buf[7] = 0x40;
  Can1.write(outgoing);
}
void send_510h(void)   //Sleep preventer / network management 3
{
  CAN_message_t outgoing;
  outgoing.id = 0x510;
  outgoing.flags.extended = false;
  outgoing.len = 8;   
  outgoing.buf[0] = 0x40;
  outgoing.buf[1] = 0x10;
  outgoing.buf[2] = 0x20;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0x00;
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0x00;
  outgoing.buf[7] = 0x00;
  Can1.write(outgoing);
}
void send_3E8h(void)  //OBD reset request
{
  CAN_message_t outgoing;
  outgoing.id = 0x3E8;
  outgoing.flags.extended = false;
  outgoing.len = 2;               
  if (OBD) outgoing.buf[0] = 0xF1;  // no OBD reset requirement, rest invalid
  else outgoing.buf[0] = 0xFB;      // OBD reset request, rest invalid
  // outgoing.buf[0] = 0xF1;
  outgoing.buf[1] = 0xFF;      
  Can1.write(outgoing);
}
void send_3F9h(void) //R/N/P/1
{
  CAN_message_t outgoing;
  outgoing.id = 0x3F9;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  outgoing.buf[0] = 0xc0;//engine info? rex?
  outgoing.buf[1] = 0xf9;
  outgoing.buf[2] = 0x80;
  outgoing.buf[3] = 0xe0;
  outgoing.buf[4] = 0x43;
  outgoing.buf[5] = 0x3c;
  // 1=N, 2=R, 3=P, 4=NP, 5=D(1), 6=2 usw. 
  if (Drive_Dir==Drive_N) outgoing.buf[6] = 0xf5; 
  else if (Drive_Dir==Drive_R) outgoing.buf[6] = 0xf2;
  else if (Drive_Dir==Drive_F) outgoing.buf[6] = 0xf5;
  if (BMS_State!=BMS_Drive) outgoing.buf[6] = 0xf3;
  outgoing.buf[7] = 0xff;   
  Can1.write(outgoing);
}
void send_512h(void) ////network management edme
{
  CAN_message_t outgoing;
  outgoing.id = 0x512;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  outgoing.buf[0] = 0x00;//network management edme
  outgoing.buf[1] = 0x00;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0x00;
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0x00;
  outgoing.buf[7] = 0x12;  
  Can1.write(outgoing);
}
void send_560h(void) //network management kombi
{
  CAN_message_t outgoing;
  outgoing.id = 0x560;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  outgoing.buf[0] = 0x00;//network management kombi
  outgoing.buf[1] = 0x00;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x00;
  outgoing.buf[4] = 0xfe;
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0x00;
  outgoing.buf[7] = 0x60;
  Can1.write(outgoing);
}
void send_330h(void) //range info, milage display
{
  CAN_message_t outgoing;
  outgoing.id = 0x330;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  outgoing.buf[0] = 0xa8;//range info, milage display
  outgoing.buf[1] = 0x86;
  outgoing.buf[2] = 0x01;
  outgoing.buf[3] = 0x02;
  outgoing.buf[4] = 0x00;
  outgoing.buf[5] = 0x05;
  outgoing.buf[6] = 0xac;
  outgoing.buf[7] = 0x03;
  Can1.write(outgoing);
}
void send_397h(void) //obd msg
{
  CAN_message_t outgoing;
  outgoing.id = 0x397;
  outgoing.flags.extended = false;
  outgoing.len = 7;
  outgoing.buf[0] = 0x00;//obd msg
  outgoing.buf[1] = 0x2a;
  outgoing.buf[2] = 0x00;
  outgoing.buf[3] = 0x6c;
  outgoing.buf[4] = 0x0f;
  outgoing.buf[5] = 0x55;
  outgoing.buf[6] = 0x00;
  Can1.write(outgoing);
}
//every 1s
void send_328h(void)   //rtc
{
  ctr_secs_LIMsend_328h++;
  CAN_message_t outgoing;
  outgoing.id = 0x328;
  outgoing.flags.extended = false;
  outgoing.len = 6;    
  outgoing.buf[0] = ctr_secs_LIMsend_328h;            //rtc msg. needs to be every 1 sec. first 32 bits are 1 second wrap counter
  outgoing.buf[1] = ctr_secs_LIMsend_328h<<8;
  outgoing.buf[2] = ctr_secs_LIMsend_328h<<16;
  outgoing.buf[3] = ctr_secs_LIMsend_328h<<24;
  outgoing.buf[4] = 0x87;                     //day counter 16 bit.
  outgoing.buf[5] = 0x1e;
  Can1.write(outgoing);
}
//every 4s
void send_2FCh(void)   //Doors locked or unlocked, hood open or closed?
{
  CAN_message_t outgoing;
  outgoing.id = 0x2FC;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  // if (Flap_Status==1) outgoing.buf[0]=0x81;
  // else if (Flap_Status==2) outgoing.buf[0]=0x80;
  // else outgoing.buf[0]=0x8f;
  outgoing.buf[0] = Flap_Status;    //0x81=flap unlock, 0x80=flap lock.
  outgoing.buf[1] = 0x00;
  // outgoing.buf[2] = 0x04;
  outgoing.buf[2] = 0b11110011;        // Engine hood closed
  outgoing.buf[3] = 0xff;
  outgoing.buf[4] = 0xff;
  outgoing.buf[5] = 0xff;
  outgoing.buf[6] = 0xff;
  outgoing.buf[7] = 0xff;
  Can1.write(outgoing);
}
void send_2A0h(void)   //central locking
{
  CAN_message_t outgoing;
  outgoing.id = 0x2A0;
  outgoing.flags.extended = false;
  outgoing.len = 8;
  // outgoing.buf[0] = 0x88;//central locking
  outgoing.buf[0] = (byte)((0xF << 4) | (Flap_Ctrl)); 
  outgoing.buf[1] = 0x88;
  // outgoing.buf[2] = 0xf8;
  outgoing.buf[2] = (byte)((0xF << 4) | (Flap_Ctrl));
  outgoing.buf[3] = 0x0f;
  outgoing.buf[4] = 0xff;
  outgoing.buf[5] = 0xff;
  outgoing.buf[6] = 0xff;
  outgoing.buf[7] = 0xff;
  Can1.write(outgoing);
}
void send_3A0h(void)   //Energy status, error memory lock
{
  CAN_message_t outgoing;
  outgoing.id = 0x3A0;
  outgoing.flags.extended = false;
  outgoing.len = 8;               
  outgoing.buf[0] = 0xFF;      // //vehicle condition
  outgoing.buf[1] = 0xFF;      // invalid
  outgoing.buf[2] = 0xC0;      // Energy status "good"
  outgoing.buf[3] = 0xFF;      // invalid
  outgoing.buf[4] = 0xFF;      // invalid
  outgoing.buf[5] = 0xFF;      // invalid
  outgoing.buf[6] = 0xFF;      // invalid
  outgoing.buf[7] = 0xFC;      // Error memory released
  Can1.write(outgoing); 
}

void send_ElconCharger_CtrlReq(void)
{
  ElconCharger_AmpReq = (uint16_t) (LIM_ACSE_I_Avbl_Grid*10); //Get AC current available at the EVSE in 0.1A increments
  ElconCharger_AmpReq = (uint16_t) (ElconCharger_AmpReq/ElconCharger_CurrentConst); //convert AC current from EVSE to DC current
  //SOC>80%
  if (BMS_SOC>800) {ElconCharger_AmpReq=ElconCharger_CurrentReqMin;}
  //SOC>=100%
  if (BMS_SOC>=1000){ElconCharger_AmpReq=0x00;}
  //BMS current limit should prevail
  if (ElconCharger_AmpReq>BMS_ChargeAmpLim){ElconCharger_AmpReq=BMS_ChargeAmpLim;}
  //Do not exceet Elcon max charging current
  if (ElconCharger_AmpReq>ElconCharger_CurrentReqMax){ElconCharger_AmpReq=ElconCharger_CurrentReqMax;}
  //Do not exceet Elcon min charging current -> quit charging
  if (ElconCharger_AmpReq<ElconCharger_CurrentReqMin){ElconCharger_AmpReq=0;}

  CAN_message_t outgoing;
  outgoing.id = canID_ElconChargerCtrl;
  outgoing.flags.extended = true;
  outgoing.len = 8;
  outgoing.buf[0] = (uint8_t) (BMS_ChargeVoltLim >> 8);
  outgoing.buf[1] = (uint8_t) (BMS_ChargeVoltLim & 0xff);
  outgoing.buf[2] = (uint8_t) (ElconCharger_AmpReq >> 8);
  outgoing.buf[3] = (uint8_t) (ElconCharger_AmpReq & 0xff);
  if (LIM_SM==24 || LIM_SM==25)
    {
      if (ElconCharger_AmpReq>0){outgoing.buf[4] = ChargeEnd_No;} //start charger
      else {outgoing.buf[4] = ChargeEnd_Yes;} //stop charger
    }
  outgoing.buf[5] = 0x00;
  outgoing.buf[6] = 0x00;
  outgoing.buf[7] = 0x00;
  Can2.write(outgoing);   
  ElconCharger_CtrlReq=false;  
}
// CHARGING---------------
// AC Charging
void AC_Charge(void)
{
  if (LIM_SM!=20) LIM_SM=20;
  if (LIM_SM==20) {Timeout=millis(); LIM_SM=21;}
  if (LIM_SM==21) TimeoutExit (2000);
  if (LIM_SM==21 && LIM_Charger_Type==1) //wait for charging mode AC Typ1; 
    {
      DC_Charge_Phase=DCChargePhase_Standby;
      CCS_Contactor_Ctrl=0x0; //dc contactor mode 0 in AC
      I_Target_DCCharge=0;//ccs current request zero
      ctr_mins_LIMsend_EOC=0xFE;//end of charge counter (minutes)
      ChargeReq=ChargeReq_Charge;
      ChargeReady=ChargeReady_Yes;
      ChargePower=6500/25;//approx 6.5kw ac
      ChargeStatus=Charge_Initialisation;
      LIM_SM=22; 
      Timeout=millis();
    }
  if (LIM_SM==22) TimeoutExit (2000);
  if (LIM_SM==22)
  {
    if (BMS_ChargeAmpLim>=ElconCharger_CurrentReqMin && LIM_ACSE_I_Avbl_Grid>0 && LIM_ACSE_I_Avbl_Grid<253)//ensure BMS charge current and EVSE charge current are within limits
      {
        Timeout=millis();
        LIM_SM=23;
      } 
  }
  if (LIM_SM==23) TimeoutExit (10000); // Give LIM time to respond with charging enable
  // if (LIM_SM==23 && LIM_Chg_Enable==1)
  if (LIM_SM==23)
    {
      ChargeStatus=Charge_Active;   //Charging status = charging active
      LIM_SM = 24;
      Timeout = millis();
    }
  if (LIM_SM==24) TimeoutExit (10000); // Give Charger time to start talking back and set the voltage
  if (LIM_SM==24 && ElconCharger_InVoltError==0) {LIM_SM=25;}  //waiting for the charger to start sending feedback
  if (LIM_SM==25)  //waiting for charging to finish
    {
      Chg_Timers(); //charging timers
      if (ElconCharger_HardwareError!=0 || ElconCharger_TempError!=0 || ElconCharger_InVoltError!=0 || ElconCharger_BatVoltError!=0 || BMS_State==BMS_Error || BMS_State==BMS_Drive) 
        {
          LIM_SM = 50; 
          ChargeStatus=Charge_Error;
        }
      if (BMS_State==BMS_Ready || pbStopChargePressed || LIM_Stat_Unloc_But==1) {LIM_SM = 26;}
    } 
  if (LIM_SM==26)  //end of charging
    {
      ChargeStatus=Charge_Completed;
      ChargeReady=ChargeReady_No;
      LIM_SM = 41;
    }
}
void TimeoutExit(int Time)
{
  if (millis() > Timeout + Time) 
    {
      ChargeStatus=Charge_Error;
      LIM_SM=50;
      ChargeReady=ChargeReady_No;
    }
}
void Everything_Off(void)
{  
  DC_Charge_Phase=DCChargePhase_Standby;
  CCS_Contactor_Ctrl=0x0; //dc contactor mode 0 in AC
  I_Target_DCCharge=0;//ccs current request zero
  ctr_mins_LIMsend_EOC = 0;
  ChargeReq=ChargeReq_EndCharge;
  ChargeReady=ChargeReady_No;
  ChargePower=0;
  ChargeEnd=ChargeEnd_Yes;
  DCChargeEnd=DCChargeEnd_Yes;
  ElconCharger_AmpReq=0;
  //reset the obtained EVSE limits
  LIM_DCSE_I_Current=0xFFFF;
  LIM_DCSE_V_Current=0xFFFF;
}

// DC Charging
void DC_Charge(void)
{
  if(LIM_SM>19)LIM_SM=0;//clear from  ac charge mode
  /*

  0=no pilot
  1=10-96%PWM not charge ready
  2=10-96%PWM charge ready
  3=error
  4=5% not charge ready
  5=5% charge ready
  6=pilot static

  */
  switch(LIM_SM)
    {
      case 0:
        {
          if (!LIM_SM_flag)
            {
              LIM_SM_flag=true; 
              millis_ctr_LIM_SM = millis();
            }
          DC_Charge_Phase=DCChargePhase_Standby;
          CCS_Contactor_Ctrl=0x0; //dc contactor mode control required in DC
          I_Target_DCCharge=0;
          ctr_mins_LIMsend_EOC=0x00;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_No;
          ChargePower=0;//0 power
          CCS_ChargeAmpReq=0;//No current
          if (millis() > (millis_ctr_LIM_SM + 2000))
            {
              LIM_SM=1; //next state after 2 secs
              millis_ctr_LIM_SM = millis();
              LIM_SM_flag=false;
            }
          break;
        }
      case 1:
        {
          DC_Charge_Phase=DCChargePhase_Initialisation;
          CCS_Contactor_Ctrl=0x0; //dc contactor mode control required in DC
          I_Target_DCCharge=0;
          ctr_mins_LIMsend_EOC=0x00;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_No;
          ChargePower=0;//0 power
          CCS_ChargeAmpReq=0;//No current
          if(LIM_Stat_Pilot == 6 || millis() > (millis_ctr_LIM_SM + 60000)) {LIM_SM=0;} //Reset to state 0 if we get a static pilot or this step is >60s
          if(LIM_Charger_Type==0x04 || LIM_Charger_Type==0x08 || LIM_Charger_Type==0x09) // //DC-Typ1 or DC-Type2
            {	
              // if (LIM_DCSE_I_Current<0xFFFF && LIM_DCSE_V_Current<0xFFFF)
                // {
                  if(millis() > (millis_ctr_LIM_SM + 2500))//2 secs efacec critical! 20 works. 50 does not.
                    {
                      LIM_SM=2;
                      millis_ctr_LIM_SM = millis();
                    }
                // }
            }
          break;
        }
      case 2:
        {
          //
          DC_Charge_Phase=DCChargePhase_CableTest;
          CCS_Contactor_Ctrl=0x0; //dc contactor mode control required in DC
          I_Target_DCCharge=0;
          ctr_mins_LIMsend_EOC=0x1E;//end of charge timer 30 mins
          ctr_secs_LIMsend_BulkSOC=1800; //Set bulk SOC timer to 30 minutes.
          ctr_secs_LIMsend_FullSOC=2400; //Set full SOC timer to 40 minutes.
          // Timer_1Sec=1000;   //Load the 1 second loop counter. 5 loops=1sec.
          Timer_60Sec=60;   //Load the 60 second loop counter.
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower=44000/25;//44kw approx power
          CCS_ChargeAmpReq=0;//No current
          if(LIM_V_CP_DC>0)
            {
              LIM_SM=3; //we wait for the contactor voltage to rise before hitting next state.
              millis_ctr_LIM_SM = millis();
            }
          break;
        }
      case 3:
        {
          DC_Charge_Phase=DCChargePhase_CableTest;
          CCS_Contactor_Ctrl=0x0; //dc contactor mode control required in DC
          I_Target_DCCharge=0;
          // EOC_Time=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower=44000/25;//39kw approx power
          CCS_ChargeAmpReq=0;//No current
          if(LIM_V_CP_DC<=50) //we wait for the contactor voltage to drop under 50v to indicate end of cable test
            {	
            if(millis() > (millis_ctr_LIM_SM + 2000))
              {
                if(LIM_DCSE_Stat_Iso==0x1) LIM_SM=4; //next state after 2 secs if we have valid iso test
                millis_ctr_LIM_SM = millis();
              }
            }
          break;
        }
      case 4:
        {
          DC_Charge_Phase=DCChargePhase_Subpoena; //precharge phase in this state
          CCS_Contactor_Ctrl= 0x0;                   //dc contactor mode control required in DC
          I_Target_DCCharge = 0;                        
          // EOC_Time=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower = 44000 / 25; //49kw approx power
          CCS_ChargeAmpReq = 0;        //No current

          if ((ISA_BatVolt/10 - LIM_V_CP_DC) < 20)
            {
              if(millis() > (millis_ctr_LIM_SM + 2000)){LIM_SM=5;} //we wait for the contactor voltage to be 20v or less diff to main batt v. must last for 2+ seconds
            }
          else {millis_ctr_LIM_SM = millis();}// If the contactor voltage wanders out of range start again

          break;
        }
      case 5:
        {
          //precharge phase in this state but voltage close enough to close contactors
          DC_Charge_Phase=DCChargePhase_Subpoena;
          CCS_Contactor_Ctrl = 0x2;                   //dc contactor closed
          I_Target_DCCharge = 0;                        
          // EOC_Time=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower = 44000 / 25; //49kw approx power
          CCS_ChargeAmpReq = 0;        //No current
          
          // Once the contactors report as closed we're OK to proceed to energy transfer
          if (((LIM_Stat_DC_Rel>>0)&1)==1) //LIM reports both contactors closed
            {
              LIM_SM=6;
              millis_ctr_LIM_SM = millis();
              millis_ctr_LIM_SOC = millis(); //update SOC hearbeat 
            }
          break;
        }
      case 6:
        {
          DC_Charge_Phase=DCChargePhase_EnergyTransfer;
          CCS_Contactor_Ctrl=0x2; //dc contactor to close mode
          //FC_Cur=Param::GetInt(Param::CCS_ICmd);//ccs manual control
          I_Target_DCCharge=CCS_ChargeAmpReq;
          CCS_Pwr_Con(); //ccs power control subroutine
          Chg_Timers();   //Handle remaining time timers.
          //  EOC_Time=0x1E;//end of charge timer
          ChargeStatus=Charge_Active;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower=44000/25;//49kw approx power
          //we chill out here charging.

          if((pbStopChargePressed)||LIM_DCSE_Stat_Chg==0x02||LIM_Stat_Unloc_But==1)//if we have a request to terminate from the button or the evse then move to next state.
            {
              I_Target_DCCharge=0;//set current to 0
              LIM_SM=7; //move to state 7 (shutdown)
              millis_ctr_LIM_SM = millis();
            }
          break;
        }
      case 7:    //shutdown state
        {
          DC_Charge_Phase=DCChargePhase_Shutdown;
          CCS_Contactor_Ctrl=0x2; //dc contactor to close mode
          I_Target_DCCharge=0;//current command to 0
          ctr_mins_LIMsend_EOC=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_Yes;
          ChargePower=44000/25;//49kw approx power
          if(millis() > (millis_ctr_LIM_SM + 1000)) //wait 1 seconds
            {
              LIM_SM=8; //next state after 2 secs
              millis_ctr_LIM_SM = millis();
            }
          break;
        }
      case 8:    //shutdown state
        {
          DC_Charge_Phase=DCChargePhase_Shutdown;
          CCS_Contactor_Ctrl=0x1; //dc contactor to open with diag mode
          I_Target_DCCharge=0;//current command to 0
          ctr_mins_LIMsend_EOC=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_Charge;
          ChargeReady=ChargeReady_No;
          ChargePower=44000/25;//49kw approx power
          if(LIM_V_CP_DC==0) //we wait for the contactor voltage to return to 0 to indicate contactors open
            {
            if(millis() > (millis_ctr_LIM_SM + 1000))
              {
                LIM_SM=9; //next state after 1 secs
                millis_ctr_LIM_SM = millis();
              }
            }
          break;
        }
      case 9:    //shutdown state
        {
          DC_Charge_Phase=DCChargePhase_Standby;
          CCS_Contactor_Ctrl=0x0; //dc contactor to open mode
          I_Target_DCCharge=0;//current command to 0
          ctr_mins_LIMsend_EOC=0x1E;//end of charge timer
          ChargeStatus=Charge_Initialisation;
          ChargeReq=ChargeReq_EndCharge;
          ChargeReady=ChargeReady_No;
          ChargePower=0;//0 power
          LIM_SM=40; //exit state
          break;
        }
    }
}

void CCS_Pwr_Con(void)    //here we control ccs charging during state 6.
{
    // Convert BMS charge current to 8bit
    uint8_t temp_BMS_ChargeAmpLim = (uint8_t) ((BMS_ChargeAmpLim/10) & 0x00FF);
    CCS_ChargeAmpReq = temp_BMS_ChargeAmpLim;
    if (CCS_ChargeAmpReq>125)CCS_ChargeAmpReq=125; //never exceed 125amps
    if (CCS_ChargeAmpReq>LIM_DCSE_I_Avbl)CCS_ChargeAmpReq=LIM_DCSE_I_Avbl; //never exceed EVSE available current
    if (CCS_ChargeAmpReq>250)CCS_ChargeAmpReq=0; //crude way to prevent rollover
    //raise amps
    if ((ISA_BatVolt<BMS_ChargeVoltLim) && (LIM_DCSE_I_LIM_Reached==0x0) && (LIM_DCSE_P_LIM_Reached==0x0))CCS_ChargeAmpReq++;//increment if voltage lower than setpoint and power and current limts not set from charger.
    if(ISA_BatVolt>BMS_ChargeVoltLim)CCS_ChargeAmpReq--;//decrement if voltage  greater than setpoint.
    //lower amps
    if(LIM_DCSE_I_LIM_Reached==0x1)CCS_ChargeAmpReq--;//decrement if current limit flag is set
    if(LIM_DCSE_P_LIM_Reached==0x1)CCS_ChargeAmpReq--;//decrement if Power limit flag is set
    //SOC>80%
    if (BMS_SOC>800 && CCS_ChargeAmpReq>5) {CCS_ChargeAmpReq=5;}
    //SOC>=100%
    if (BMS_SOC>=1000){CCS_ChargeAmpReq=0;}
    // //BMS charge current limit for CCS
    if(CCS_ChargeAmpReq>temp_BMS_ChargeAmpLim) CCS_ChargeAmpReq = temp_BMS_ChargeAmpLim;
    // if(BMS_SOC>770 && CCS_ChargeAmpReq>ElconCharger_CurrentReqMax && CCS_ChargeAmpReq>LIM_DCSE_I_Min_Avbl) 
    //   {
    //     if (LIM_DCSE_I_Min_Avbl>ElconCharger_CurrentReqMax) CCS_ChargeAmpReq=LIM_DCSE_I_Min_Avbl;
    //     else CCS_ChargeAmpReq=ElconCharger_CurrentReqMax;
    //   }
}
void Chg_Timers(void)
{
  float temp_ctr_mins_LIMsend_EOC;//=0xFE;
  float temp_ISA_BatAmp=ISA_BatAmp-8192;

  // if (LIM_SM==25) //AC charging, charging is active
  //   {
  //     //timer left to full charge in minutes
  //     temp_ctr_mins_LIMsend_EOC=(float) (BMS_SOC/1000.0); //BMS_SOC is in 0.1% scale, hence /1000% not /100%
  //     temp_ctr_mins_LIMsend_EOC=(float) (1-temp_ctr_mins_LIMsend_EOC);
  //     temp_ctr_mins_LIMsend_EOC=(float) (BMS_CapacityAh*temp_ctr_mins_LIMsend_EOC);
  //     temp_ctr_mins_LIMsend_EOC=(float) (temp_ctr_mins_LIMsend_EOC*60.0); //hrs to mins *60
  //     temp_ctr_mins_LIMsend_EOC=(float) (temp_ctr_mins_LIMsend_EOC*10.0/(8192-ISA_BatAmp)); //ISA amps are in 0.1A scale, hence *10
  //     if (temp_ctr_mins_LIMsend_EOC>0xFE) ctr_mins_LIMsend_EOC=0xFE;
  //     else ctr_mins_LIMsend_EOC= (uint8_t) temp_ctr_mins_LIMsend_EOC;
  //   }
  // else if (LIM_SM==6) //DC charging, charging is active
  //   {
  //     // if(millis() > millis_ctr_LIM_SOC + 1000)
  //     //   {
  //     //     ctr_secs_LIMsend_BulkSOC--;    //Decrement timers.
  //     //     ctr_secs_LIMsend_FullSOC--;
  //     //     Timer_60Sec--;  //decrement the 1 minute counter
  //     //     if(Timer_60Sec==0)
  //     //       {
  //     //           Timer_60Sec=60;
  //     //           ctr_mins_LIMsend_EOC--;    //decrement end of charge minutes timer
  //     //       }
  //     //     millis_ctr_LIM_SOC = millis(); //update SOC hearbeat 
  //     //   }
  //     //timer left to charge
  //     temp_ctr_mins_LIMsend_EOC=(float) (BMS_SOC/1000.0); //BMS_SOC is in 0.1% scale, hence /1000% not /100%
  //     temp_ctr_mins_LIMsend_EOC=(float) (1-temp_ctr_mins_LIMsend_EOC);
  //     temp_ctr_mins_LIMsend_EOC=(float) (BMS_CapacityAh*temp_ctr_mins_LIMsend_EOC);
  //     temp_ctr_mins_LIMsend_EOC=(float) (temp_ctr_mins_LIMsend_EOC*60.0); //hrs to mins *60
  //     temp_ctr_mins_LIMsend_EOC=(float) (temp_ctr_mins_LIMsend_EOC*10.0/(8192-ISA_BatAmp)); //ISA amps are in 0.1A scale, hence *10
  //     if (temp_ctr_mins_LIMsend_EOC>0xFE) ctr_mins_LIMsend_EOC=0xFE;
  //     else ctr_mins_LIMsend_EOC= (uint8_t) temp_ctr_mins_LIMsend_EOC;
  //     if ((temp_ctr_mins_LIMsend_EOC*60)>0xFFFE) ctr_secs_LIMsend_FullSOC=0xFFFE;
  //     else ctr_secs_LIMsend_FullSOC= (uint16_t) temp_ctr_mins_LIMsend_EOC*60;
  //     ctr_secs_LIMsend_BulkSOC=(uint16_t) temp_ctr_mins_LIMsend_EOC*48; //80% * 60 = 48
  //   }
  if (LIM_SM==6 || LIM_SM==25) //DC or AC charging, energy transfer
    {
      //timer left to charge
      temp_ctr_mins_LIMsend_EOC=(float) (BMS_SOC/1000.0); //BMS_SOC is in 0.1% scale, hence /1000% not /100%
      temp_ctr_mins_LIMsend_EOC=(float) (1-temp_ctr_mins_LIMsend_EOC);
      temp_ctr_mins_LIMsend_EOC=(float) (BMS_CapacityAh*temp_ctr_mins_LIMsend_EOC);
      temp_ctr_mins_LIMsend_EOC=(float) (temp_ctr_mins_LIMsend_EOC*60.0); //hrs to mins *60
      temp_ctr_mins_LIMsend_EOC=(float) temp_ctr_mins_LIMsend_EOC*10.0; //ISA amps are in 0.1A scale, hence *10
      temp_ctr_mins_LIMsend_EOC=(float) temp_ctr_mins_LIMsend_EOC/temp_ISA_BatAmp;
      if (temp_ctr_mins_LIMsend_EOC>0xFE) ctr_mins_LIMsend_EOC=0xFE;
      else ctr_mins_LIMsend_EOC= (uint8_t) temp_ctr_mins_LIMsend_EOC;
      if ((temp_ctr_mins_LIMsend_EOC*60)>0xFFFE) ctr_secs_LIMsend_FullSOC=0xFFFE;
      else ctr_secs_LIMsend_FullSOC= (uint16_t) temp_ctr_mins_LIMsend_EOC*60;
      ctr_secs_LIMsend_BulkSOC=(uint16_t) temp_ctr_mins_LIMsend_EOC*48; //80% * 60 = 48
    }
}

void DC_test_on(void)
{
  if (Flap_Status==1)
    {
    digitalWrite(flapSimulatorOut, LOW); //Close flap
    OBD=true;                   // OBD error reset request
    Ignition_Status_Sim=0x8A;                 // Ignition on
    Flap_Status=2;                 //Flap locked
    //U_ZKr_Akt = U_Bat_Akt; // Close DC_Main
    //  event_CNT[4]=3; 
    //Status=50
    }
}
void DC_test_off(void)
{
  if (Flap_Status==2)
    {
    Flap_Status=1;                   //Flap unlocked
    OBD=false;
    Ignition_Status_Sim=0x86;                  //6 = ignition off
    //  event_CNT[4]=3; 
    digitalWrite(flapSimulatorOut, HIGH); //Open the flap
    //U_ZKr_Akt =0; // Open DC_Main
    //  Status=0;
    }
}


// MAIN -----------------------------------
void loop() 
{
if (Serial.available() > 0)
{
  incomingByte = Serial.read(); // read the incoming byte:
  menu();
}
//CPU------------------------------------------------------------------------------------------------------
VCUtempC = InternalTemperature.readTemperatureC();
// WATCHDOG-------------------------------------------------------------------------------------------------
if ((millis() > (Debug_Stamp + 1000))) 
  {
    if(simulation_menu) {menu();}
    else {Debug();}
  }
if (millis() > (LIM_watchdog + 1000)) {Set_ZeroLIM();}
if (millis() > (ISA_watchdog + 1000)) {Set_ZeroISA();}
if (millis() > (BMS_watchdog + 2000)) {Set_ZeroBMS();}
if (millis() > (Drive_watchdog + 1000)) {Set_ZeroDrive();}
if (millis() > (IBooster_watchdog + 2000)) {Set_ZeroIBooster();}
if (millis() > (LED_watchdog + 1000)) {digitalToggle(LED_BUILTIN);LED_watchdog=millis();}
if (millis() > (Elcon_watchdog + 2000)) {Set_ZeroElcon();}

//CANbus MONITOR-------------------------------------------------------------------------------------
if (Can1.read(msg))
  {
    if (msg.id==0x272 || msg.id==0x2EF || msg.id==0x2B2 || msg.id==0x29E || msg.id==0x390 || msg.id==0x337 || msg.id==0x3B4) {Check_LIM(msg);}
    if (msg.id==canID_BMSLimits || msg.id==canID_BMSInfo || msg.id==canID_BMSLowHigh || msg.id==canID_BMSSOC || msg.id==canID_BMSStatus || msg.id==canID_BMSAlarmsWarnings) {Check_BMS(msg);}
    if (msg.id>=0x521 && msg.id<=0x524) {Check_ISA(msg);}
    if (msg.id==canID_Drive_Stat) {Check_Drive(msg);}
    if (msg.id==canID_stsIBooster) {Check_IBooster(msg);}
  }
if (Can2.read(msg)){if (msg.id==canID_ElconChargerFback || msg.id==canID_ElconDCDCFback) {Check_Elcon(msg);}}
if (ElconCharger_CtrlReq && BMS_State == BMS_Charge) {send_ElconCharger_CtrlReq();}

//IGNITION SWITCH
if(digitalRead(inIgnition)) 
  {
    swIgnition=false;
    swIgnitionDelayOn=false;
    IgnitionDelayOn_Stamp=millis();
    if (millis() > (IgnitionDelayOff_Stamp + 3000)){swIgnitionDelayOff=true;}
  }
else 
  {
    swIgnition=true;
    swIgnitionDelayOff=false;
    IgnitionDelayOff_Stamp=millis();
    if (millis() > (IgnitionDelayOn_Stamp + 3000)){swIgnitionDelayOn=true;}
  }

//START CTRL
if (swIgnition)    
  {
    if (Drive_Status != Drive_Stat_Invalid && Drive_OpMode != Drive_OPMODE_Invalid && Drive_OpMode == Drive_OPMODE_Off && Drive_Status == Drive_Stat_WaitStart && IBST_driverBrakeApply!=IBST_Invalid && IBST_iBoosterStatus!=IBST_Invalid && IBST_internalState!=IBST_Invalid && IBST_internalState==LOCAL_BRAKE_REQUEST && IBST_driverBrakeApply==DRIVER_APPLYING_BRAKES && Drive_Cmd_StartPerm==true) 
    {
      //send start command via CAN
        Drive_Cmd_Start=true;
        // send_Drive_Cmd();
        Drive_Cmd_Start_Watchdog=millis();
        Drive_Cmd_StartPerm=false;
    }
  }
else 
  {
    if (swIgnitionDelayOff) 
      {
        Drive_Cmd_Start=false;
        // send_Drive_Cmd();
        Drive_Cmd_StartPerm=true;
      }
  }

if (Drive_Cmd_Start)
  {
    if ((millis() > (Drive_Cmd_Start_Watchdog + 5000)) || (Drive_Status != Drive_Stat_Invalid && Drive_OpMode != Drive_OPMODE_Invalid && Drive_OpMode == Drive_OPMODE_Run))
      {
        Drive_Cmd_Start=false;
        // send_Drive_Cmd();
        Drive_Cmd_StartPerm=false;
      }
  }
  
//DCDC CTRL-------------------------------------------------------------------------------------
// Delayed start and continuous enable signal for DCDC
if ((ElconDCDC_Sts==1 && (ElconDCDC_StopError!=0 || ElconDCDC_HardwareError!=0 || ElconDCDC_Ready!=1 || ElconDCDC_OutOverAmp!=0 || ElconDCDC_InUnderVolt!=0 || ElconDCDC_OutUnderVolt!=0 || ElconDCDC_OutOverVolt!=0 || ElconDCDC_OverTemp!=0)))
    {DCDC_error=true;}
else {DCDC_error=false;}

if (swIgnition && Drive_OpMode==Drive_OPMODE_Run && BMS_State != BMS_Boot && BMS_State != BMS_Error && !DCDC_error) 
  {
    if (millis() > (DCDCPre_Stamp + 5000)) {digitalWrite(outDCDCenable, HIGH);}//DCDC enabled
  }
else 
  {
    digitalWrite(outDCDCenable, LOW); //DCDC disabled
    DCDCPre_Stamp=millis();
  }

//FAN AND PUMP-------------------------------------------------------------------------------------
if (swIgnitionDelayOn) //ignition on?
  {          
    if(!coolantOverride) //auto coolant control if simulation isn't enabled
      {
        // pwmBPumpVarTemp=pwmPumpHigh_lowFlow;
        pwmMPumpVarTemp=pwmPumpHigh_hiFlow;
        // pwmBFanDutyTemp =pwmFan_mid;
        pwmMFanDutyTemp =pwmFan_off;
        if (BMS_State == BMS_Drive) {pwmMFanDutyTemp =pwmFan_mid;}
        if (BMS_State == BMS_Charge)
          {
            if (Drive_HtSnkTemp<=6000) {pwmMFanDutyTemp =pwmFan_low;}
            else {pwmMFanDutyTemp =pwmFan_mid;}
          }
      }
  }
else  //ignition off?
  {
    pwmBFanDutyTemp =pwmFan_off;
    pwmMFanDutyTemp =pwmFan_off;
    pwmBFanReqDuty=pwmBFanDutyTemp ;
    pwmMFanReqDuty=pwmMFanDutyTemp ;
    pwmMPumpVarTemp=pwmPumpHigh_off;
    pwmBPumpVarTemp=pwmPumpHigh_off;
    pwmMPumpVar=pwmMPumpVarTemp;
    pwmBPumpVar=pwmBPumpVarTemp;
  }

if (pwmBFanReqDuty!=pwmBFanDutyTemp || pwmMFanReqDuty!=pwmMFanDutyTemp ||pwmMPumpVar!=pwmMPumpVarTemp || pwmBPumpVar!=pwmBPumpVarTemp) //temp value updated by the temp threshold? => delay the change for smooth on/off control => no spike changes
  {
    if (millis() > (CoolingFanPumpDelay_Stamp + 500)) //7s delay
      {
        //battery fan
        if(pwmBFanDutyTemp <pwmFan_hi)pwmBFanDutyTemp =pwmFan_hi;
        if(pwmBFanDutyTemp >pwmFan_low)pwmBFanDutyTemp =pwmFan_off;
        pwmBFanReqDuty=pwmBFanDutyTemp ;
        //main fan
        if(pwmMFanDutyTemp <pwmFan_hi)pwmMFanDutyTemp =pwmFan_hi;
        if(pwmMFanDutyTemp >pwmFan_low)pwmMFanDutyTemp =pwmFan_off;
        pwmMFanReqDuty=pwmMFanDutyTemp ;
        // main pump
        pwmMPumpVar=pwmMPumpVarTemp;
        // battery pump
        pwmBPumpVar=pwmBPumpVarTemp;
      }
  }
else {CoolingFanPumpDelay_Stamp=millis();}

//continuous fan pwm update
//battery fan
analogWrite(pwmOutFanBat,pwmBFanReqDuty);
//main fan
analogWrite(pwmOutFanMain,pwmMFanReqDuty);

//continuous pumps pwm update - 2Hz base freq - cannot be created via hardware
//Motor
if (pwmMPumpHigh)
  {
    if (millis() > (PWM_MPump_Stamp + pwmMPumpVar)) 
        {
          digitalWriteFast(pwmOutPumpMain, HIGH);
          pwmMPumpHigh=false;
          PWM_MPump_Stamp=millis();
        }
  }
else
  {
    if (millis() > (PWM_MPump_Stamp + (pwmPumpPeriod-pwmMPumpVar))) 
        {
          digitalWriteFast(pwmOutPumpMain, LOW);
          pwmMPumpHigh=true;
          PWM_MPump_Stamp=millis();
        }
  }
//Battery
if (pwmBPumpHigh)
  {
    if (millis() > (PWM_BPump_Stamp + pwmBPumpVar)) 
        {
          digitalWriteFast(pwmOutPumpBat, HIGH);
          pwmBPumpHigh=false;
          PWM_BPump_Stamp=millis();
        }
  }
else
  {
    if (millis() > (PWM_BPump_Stamp + (pwmPumpPeriod-pwmBPumpVar))) 
        {
          digitalWriteFast(pwmOutPumpBat, LOW);
          pwmBPumpHigh=true;
          PWM_BPump_Stamp=millis();
        }
  }

// CHARGING CTRL-------------------------------------------------------------------------------------
//"stop charge" button
if(digitalRead(inStopCharge)) {pbStopChargePressed=false;}
else  {pbStopChargePressed=true;}

// monitor charging request
if (LIM_Stat_Line_Plg!=1)  //no cable plugged in
  {
    if (Flap_Status==1)
      {
        LIM_SM=0; 
        ChargeStatus=Charge_NotReady;
        digitalWrite(outBMSChargeReq,LOW);//tell BMS to exit charge mode - LOW on BMS charge enable input
        Everything_Off();
      }
  }
else {digitalWrite(outBMSChargeReq,HIGH);} //tell BMS to go into charge mode - HIGH on BMS charge enable input}

// if (BMS_State == BMS_Charge) {OBD=false;}
// else {OBD=true;}

// DC isolation test, LIM contactors check on car start and whenever BMS = Ready (Idle state) - is test=fail, run LIM contactors check
if (swIgnitionDelayOn && Drive_OpMode == Drive_OPMODE_Run && BMS_State == BMS_Ready)
  {
    if (((LIM_Stat_DC_Rel>>2)&7)!=0 && LIM_Stat_Pos_Flap==1) {DC_test_on();}
    if (((LIM_Stat_DC_Rel>>2)&7)==0) {DC_test_off();}
  }
//Charging
if ((BMS_CellsTempMin-273)>=5 && (BMS_CellsTempMax-273)<=40) //charge under certain battery temp conditions
  {
    if (swIgnition && Drive_OpMode == Drive_OPMODE_Run && BMS_State==BMS_Charge)
      {
        // Charge type selection: 
        //AC
        if ((LIM_Stat_Pilot == 1 || LIM_Stat_Pilot == 2) && LIM_Stat_Line_Plg==1 && Flap_Status==1) {AC_Charge();}
        //DC
        if ((LIM_Stat_Pilot == 4 || LIM_Stat_Pilot == 5) && LIM_Stat_Line_Plg==1 && Flap_Status==1) {DC_Charge();}
      }
  }
else
  {
    LIM_SM=40; 
    ChargeStatus=Charge_NotReady;
    Everything_Off();  
  }
//charge error
if ((LIM_Stat_Pilot == 0 || LIM_Stat_Pilot == 3 || LIM_Stat_Pilot == 6 || LIM_Stat_Pilot == 7) && LIM_Stat_Line_Plg==1) // Pilot error
  { 
    if (LIM_SM > 0 && LIM_SM < 40) 
    { 
      LIM_SM=50;
      ChargeStatus=Charge_Error;
    }
  }
if (LIM_SM>=40 && LIM_SM<=50){Everything_Off();}  
}
