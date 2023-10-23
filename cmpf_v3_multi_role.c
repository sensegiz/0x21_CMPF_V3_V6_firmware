/******************************************************************************

@file  multi_role.c

@brief This file contains the multi_role sample application for use
with the CC2650 Bluetooth Low Energy Protocol Stack.

Group: WCS, BTS
Target Device: cc2640r2

******************************************************************************

 Copyright (c) 2013-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

//#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "multi.h"
#include "battservice.h"
#include "battservice.c"

#include "board.h"
#include "multi_role.h"
#include "math.h"
#include "ExtFlash.h"
#include "lsm6ds3.h"
#include "aon_batmon.h"
#include "bsp_i2c.h"
#include "Watchdog.h"
#include "utc_clock.h"
#include "time_clock.h"

#include <profiles/oad/cc26xx/oad.h>
#include <profiles/oad/cc26xx/oad_image_header.h>
// Needed for HAL_SYSTEM_RESET()
#include "hal_mcu.h"
#include "ble_user_config.h"

/*********************************************************************
* CONSTANTS
*/

// Maximum number of scan responses
// this can only be set to 15 because that is the maximum
// amount of item actions the menu module supports
#define DEFAULT_MAX_SCAN_RES                  10

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters
#define DEFAULT_CONN_INT                      42//200
#define DEFAULT_CONN_TIMEOUT                  200//1000
#define DEFAULT_CONN_LATENCY                  0

// Scan parameters
#define DEFAULT_SCAN_DURATION                 120//4000
#define DEFAULT_SCAN_WIND                     200//80//192
#define DEFAULT_SCAN_INT                      200//80//192

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_GENERAL//DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// Set desired policy to use during discovery (use values from GAP_Disc_Filter_Policies)
#define DEFAULT_DISCOVERY_WHITE_LIST          GAP_DISC_FILTER_POLICY_ALL

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1150//610
#endif

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define MR_STATE_CHANGE_EVT                  Event_Id_00
#define MR_CHAR_CHANGE_EVT                   Event_Id_04
#define MR_CONN_EVT_END_EVT                  Event_Id_05
#define TEMP_STREAM_EVT                      Event_Id_06
#define SEND_BUFFER_DATA                     Event_Id_07
#define ACK_TIMEOUT_EVT                      Event_Id_08
#define BATT_EVT                             Event_Id_09
#define DYNAMIC_ID_RESEND_EVT                Event_Id_10

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                             MR_QUEUE_EVT            | \
                                             MR_STATE_CHANGE_EVT     | \
                                             MR_CHAR_CHANGE_EVT      | \
                                             MR_CONN_EVT_END_EVT     | \
                                             TEMP_STREAM_EVT         | \
                                             SEND_BUFFER_DATA        | \
                                             ACK_TIMEOUT_EVT         | \
                                             BATT_EVT                | \
                                             DYNAMIC_ID_RESEND_EVT   | \
                                             OAD_QUEUE_EVT           | \
                                             OAD_DL_COMPLETE_EVT     | \
                                             OAD_OUT_OF_MEM_EVT)

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// address string length is an ascii character for each digit +
// an initial 0x + an ending null character
#define B_STR_ADDR_LEN       ((B_ADDR_LEN*2) + 3)

// How often to perform periodic event (in msec)
// clock periods of events in msec
#define TEMP_STREAM_PERIOD                   997
#define SEND_PERIOD                          503//1013
#define ACK_PERIOD                           1109//2013//1109
#define BATT_PERIOD                          599999 //1199999//1200007// FOR EVERY 10 min
#define DYN_ID_RESEND_PERIOD                 900007//300007
#define READ_DELAY_PERIOD                    2003

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

//variable sizes
#define GATEWAY_DATA                          6
#define BUFFER_LIMIT                          2800//3200//2000//2500
#define BUFFER_LIMIT1                         40000//3200//2000//2500
#define DEV_LIST_SIZE                         10

//external flash locations for data storing
#define sensor_loc                            0x60010//0x7B001
#define tx_power_loc                          0x60030//0x7B020
#define freq_range_loc                        0x60050//0x7B060
#define mesh_id_loc                           0x60070//0x7B060
#define extflash_addr_choice                  0x60090

//extflash locations of old extflash
#define extbuffer_loc0                        0x61100
#define extbuffer_loc(i)                      (extbuffer_loc0+(0xF00*i))
#define extbuffer_loc1                        0x70100
#define extbuffer_loc_1(i)                    (extbuffer_loc1+(0xF00*i))

//extflash locations of new extflash
#define extbuffer_loc2                        0x81000
#define extbuffer_loc_2(i)                    (extbuffer_loc2+(0xF00*i))//81000+F00*85=FDB00
#define extbuffer_loc3                        0x101000
#define extbuffer_loc_3(i)                    (extbuffer_loc3+(0xF00*i))

//#define sensor_copy_loc23                     0x7B010//0x65100//0x76890//
#define sensor_enable_loc                     0x7B020//0x62060//0x71060//0x7D020
#define set_threshold_loc                     0x7B060//0x62080//0x71080//0x7D080
#define activity_read_time_loc                0x7B080
#define time_date_loc                         0x7B100//0x62120//0x71080//0x7D080

#define write_count_store                     0x7C010//0x63070//0x72070//0x7E060
#define read_count_store                      0x7C050//0x63090//0x72190//0x7E190

#define sensor_copy_loc1                      0x7E010//0x65100//0x76890//0x7B010
#define mesh_id_copy_loc1                     0x7E050//0x65120//0x76910

#define reset_count_loc                       0x7F010//0x64080//0x73080//0x7F080
#define sensor_copy_loc2                      0x7F050//0x64100//0x73100//0x7F160
#define mesh_copy_loc2                        0x7F090//0x64120//0x73120//0x7F160

//macro for delay in msec
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))

//i2c addresses of temp and acc sensors
#define SENSOR_I2C_ADDRESS                     0x6A//lsm6ds3
#define SENSOR_I2C_ADDRESS1                    0x40//si-7006 a-20

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData;  // event data pointer
} mrEvt_t;

// discovery information
typedef struct
{
  discState_t discState;   // discovery state
  uint16_t svcStartHdl;    // service start handle
  uint16_t svcEndHdl;      // service end handle
  uint16_t charHdl;        // characteristic handle
} discInfo_t;

//structure of scanned coins
typedef struct
{
   uint8 static_id[2];
   int8 RSSI;
   uint8 DevList_index;
   bool in_range;
   uint8 addr1[6];
   uint8 dyn_id[2];
   uint16 dyn_id_16;
}dev_list;

// entry to map index to connection handle and store address string for menu module
typedef struct
{
  uint16_t connHandle;              // connection handle of an active connection
  uint8_t strAddr[B_STR_ADDR_LEN];  // memory location for menu module to store address string
} connHandleMapEntry_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

//declaration of variables
uint8_t connected_device_address[6],same_dev_address[6],mesh_id=0,mesh_id_copy=0,mesh_id_copy2=0,scanned_dyn_id[2]={0x00,0x64},gateway_count=0,Correction_count=0,one_sec_count=0,one_min_count=0,status_resend=0,coin_scan_count=0;
uint8_t sensor_id[2]={0},sensor_id_copy[2]={0},sensor_id_copy2[2]={0},last_dynamic_id=0,dynamic_id[2]={0x00,0x64},reset_count=0,time_date[5]={0xFF},no_of_scan_cycle=0,adv1;
uint16_t sensor_id_16=0,dynamic_id_16=0x0064,small_id=0x0064,self_dyn=0x0000,gateway_found=0,G_off=0,gateway_scan_count=0,dyn_id_previous_16=0x0064,scanned_dyn_id_16=0,getset_dynid_16=0,x,y,z1;
int8 RSSI,range;
uint8_t s,t,u,v,w,z,same_dyn_id=0,status,coin_state=0,adv_update=0,adv_up_ios=0;
const int8 RSSI_LIMIT = -92;
const int8 RSSI_LIMIT_1 = -70;
static uint8_t time_hr=0,time_min=0,time_sec=0,input_date=1,last_date=30,j=0;

Watchdog_Handle WatchdogHandle;

/*********************************************************************
* LOCAL VARIABLES
*/

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;

/*********************************************************************
* LOCAL VARIABLES
*/

//declaration of variables
int16_t ax_val,ay_val,az_val,ax_val1,ay_val1,az_val1,ax_val2,ay_val2,az_val2,acc_x1,acc_y1,acc_z1,sum;
int16_t gx_val,gy_val,gz_val,gx_val1,gy_val1,gz_val1;
float ax,ay,az,xa,ya,za,ax_g,ay_g,az_g,ax_g_mod,ay_g_mod,az_g_mod;
uint8_t val,ax_L,ax_H,ay_L,ay_H,az_L,az_H,gx_L,gx_H,gy_L,gy_H,gz_L,gz_H;
float g_x,g_y,g_z,g,g1,g_low_ext,g_low_ext_1,g_high_ext,g_high_ext_1;
int temp_thld_low,temp_thld_high;
uint8_t ix,jx,ig,n=0,same_device_transfer=0,ij=0,sensor_id_zero=0,sensor_id_copy_zero=0,sensor_id_copy2_zero=0;
uint8_t dynamic_data[SENSOR_LEN],buffer_ext_store=0,buff_store=0,freq_factor=0x60,dev_disconnect_central=0;
uint8_t acceData1[SENSOR_LEN],acceData2[SENSOR_LEN],acceData3[SENSOR_LEN],acceData4[SENSOR_LEN],acceData8[SENSOR_LEN],scan_dyn_list[10]={0};

static volatile bool sensorReadScheduled;
static uint16_t readcount=0,data_read_count=0,writecount=0,buffer_counter=0;

float g1_low=1,g1_high=3.375,g2_low=3.375,g_low=10,g_high=1000,g2_low_ext,g1_low_ext,g2_low_ext_2,g1_low_ext_1,check1=0;
uint8_t loc1=0,loc2=0,loc3=0,loc4=0,loc5=0,loc6=0,loc7=0,loc8=0,loc13=0,loc14=0,loc15=0,loc16=0,loc17=0,loc18=0,loc20=0,loc21=0,loc22=0;//loc=0,loc12=0
static uint8_t loc=0,loc12=0;
uint8_t coin_buffer[COIN_DATA],gateway_buffer[1][GET_DATA];
uint8_t value,simple1handle=26,simple3handle=32,addrType1,cindex,oad_in_progress = 0;
static uint8 power_set_minus_21=0,power_set_minus_18=0,power_set_minus_12=0,power_set_minus_6=0,power_set_zero=0,power_set_plus_5=1;

float neg_temp,temperature,init_temp_low=0,init_temp_high=0;
static uint8_t acc_enable=1,gyro_enable=1,temp_enable=1,hum_enable=1,temp_stream_enable=1,hum_stream_enable=1;
uint8_t dataToSend[10][COIN_DATA]={},time_data[COIN_DATA],time_data_val=0,batt_init=0,batt_count=0,batt_75=0,batt_85=0,coin_id[ID_DATA],gateway_flg_en=0;
uint16_t temp_float=0,temp_float1=0,on_flag=0;//batt_level
static uint8_t write_store[3]={0},read_store[3]={0};
uint16_t write_mid=0,read_mid=0;
static uint8_t bat_val=2;//default medium battery level
uint8_t batt_H,batt_M,batt_L,battry_drain_fast,battry_same,last_batt_level=79;
static uint16_t batt_level;

uint8_t tempdata1[SENSOR_LEN],tempdata2[SENSOR_LEN],batt_data[SENSOR_LEN];
uint8_t simplegattdata[COIN_DATA],thresholddata[GET_DATA];
uint8_t setdata[12],dummy_sensor_data[SENSOR_LEN],sensor_enable_data[6],tx_pwr_data;
int Temp_threshold_high=70,Humidity_threshold_high=95,Temp_threshold_low=5,Humidity_threshold_low=1;
uint8 temp_low=0,temp_high=0,device_name[25],scan_count=0;
uint8 moving_device_name[]="CMPF";

uint8 gateway_name[]="GATWYPF";
uint8 *ptr,data[2]={0},last_coin_packet[COIN_DATA]={0},sensor_id_index,curr_addr[GATEWAY_DATA],prev_addr[GATEWAY_DATA];
uint8 packet_to_send[COIN_DATA],packet_to_send1[GET_DATA],packet_id=0,last_coin_packet1[GET_DATA]={0};
uint8 device_length=sizeof(gateway_name)-1;
uint8 moving_device_length=sizeof(moving_device_name)-1;
uint8 dev_counter=0,res,devices_discovered=0,*peerAddr,adv_data[25],dest_index,gatdata=0,dest_addr[6];

bool readreturn,extest,writereturn;
bool readreturn1,writeretrn1;
uint8 ack=0xAC,connRole,ack_packet[ACK_DATA],dis_status,dyn_scan_start=1;
uint8_t appdata='Y',dynamic_resend=0,set_get=0,ext_fl_part=0,ext_id=0xEF,ext_id1=0;
extern uint8_t appflag;
uint8_t firmw_notifyer=1,firmware_use_case=0x33,date=27,month=10;

PIN_Handle hGpioPin;
dev_list scanned_coin_info[DEV_LIST_SIZE];
uint8 first_data_read=0,buffer_counter2=0;

uint8 dev_list_index=0,count=0,valid_coin=0,count1=0;
uint8_t temp_hum_loop=0,reset_before=0,device_connected_inc=0,ext_addr_ch_bt=1,read_rpt_start=1,acc_start=1,read_min_inc=0,read_5min=0,read_time_insec=0x0A;

static uint8_t settings_changed=0,time_received=0,min_diff=0;
static uint8_t device_connected=0;
uint32_t utc_time_server_input=0,time_total=0,time_total_dup=0;

unsigned char *BLE0 = (unsigned char*) 0x500012ED;
unsigned char *BLE1 = (unsigned char*) 0x500012EC;
unsigned char *BLE2 = (unsigned char*) 0x500012EB;
unsigned char *BLE3 = (unsigned char*) 0x500012EA;
unsigned char *BLE4 = (unsigned char*) 0x500012E9;
unsigned char *BLE5 = (unsigned char*) 0x500012E8;
unsigned char BLEADDRESS[6];

static bool isOpen = false;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
//event clocks
Clock_Struct ack_clock;
static Clock_Struct batt_clock;
static Clock_Struct dyn_id_resend_clock;
static Clock_Struct read_delay_clock;
static Clock_Struct tempstreamClock;
static Clock_Struct buffer_clock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

//san response data of coin
static uint8_t scanRspData[] =
{
 // complete name
 10,   // length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'C', 'M', 'P', 'F', '0', '0', '0', '0', '0',

 0x0C,
 GAP_ADTYPE_LOCAL_NAME_SHORT,//0x08,
 0x00,  //sensor_id0
 0x00,  //sensor_id1
 0x00,  //dyn_id0
 0x64,  //dyn_id1
 0x00,  //time_hr
 0x00,  //time_min
 0x00,  //time_sec
 0x00,  //input_date
 0x00,  //last_date
 0x00,  //mesh_id
 0x00   //reset_count
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
//advert data of coin
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x09,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
};

// pointer to allocate the connection handle map
static connHandleMapEntry_t *connHandleMap;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CMPF00000";

// Number of scan results
static uint8_t scanRes = 0;

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pointer to per connection discovery info
discInfo_t *discInfo;

// Scanning started flag
static bool scanningStarted = FALSE;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Dummy parameters to use for connection updates
gapRole_updateConnParams_t updateParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = 80,
  .maxConnInterval = 150,
  .slaveLatency = 0,
  .timeoutMultiplier = 200
};

// Connection index for mapping connection handles
static uint16_t connIndex = INVALID_CONNHANDLE;

// Maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

/*********************************************************************
* LOCAL FUNCTIONS
*/
//function declaration
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);

static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);

static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr);
static void multi_role_clockHandler(UArg arg);
static void multi_role_connEvtCB(Gap_ConnEventRpt_t *pReport);

static void multi_role_processOadWriteCB(uint8_t event, uint16_t arg);
static void multi_role_processL2CAPMsg(l2capSignalEvent_t *pMsg);

bool flashread();
void GetAddress(void);
void coin_adv(void);
void turn_on_adv(void);
void count_restore();
void external_flash_erase();
void buff_limit_erase();
void count_store();
void Sensor_init(void);
void SensorLSM6DS3_processInterrupt(void);
void WatchdogApp_Init(void);
void add_to_buffer(uint8 );
void read_from_flash(uint8_t* , int );
void DataSend(uint8*,uint8,uint8_t);

void Activity(void);
void Accl_read(void);
void read_gyro(void);
static int16_t dataconvert(int16_t);
void temp_activity();
void dynamic_id_resend();
void batt_activity();
void time_updation();
void time_start();
void time_save();

void process_adv_packet();
uint8 compare_string(uint8* ,uint8* ,uint8 );
uint8 find_dest_addr(void);
void Connect_to_coin();
void disconnect_and_update_buffer(uint8 );
void disconnect_and_update_gateway_buffer(uint8 );

void gatewaydata();
void write_set_values(uint8_t*);
void write_single_data(uint8_t);
//void write_single_data1(uint8_t);

/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);
extern void write_single_data1(uint8_t);
/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,                   // Events to be handled by the app are passed through the GAP Role here
  multi_role_paramUpdateDecisionCB      // Callback for application to decide whether to accept a param update
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

static oadTargetCBs_t multi_role_oadCBs =
{
  .pfnOadWrite = multi_role_processOadWriteCB // Write Callback.
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NOT_REGISTER       = 0,
   FOR_AOA_SCAN       = 1,
   FOR_ATT_RSP        = 2,
   FOR_AOA_SEND       = 4,
   FOR_TOF_SEND       = 8,
   FOR_OAD_SEND       = 0x10,
}connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      multi_role_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegister represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t multi_role_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(multi_role_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      multi_role_UnRegistertToAllConnectionEvent()
 *
 * @brief   Un register  connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t multi_role_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(multi_role_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for multi_role.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
    //data read from extflash after start
    uint8_t i;
    flashread();
    GetAddress();
    coin_adv();
    if (sensor_id_zero==1)
    {
        write_single_data(sensor_id[0]);
    }
    else if (sensor_id_copy_zero==1)
    {
        sensor_id_copy[0]=sensor_id[0];
        sensor_id_copy[1]=sensor_id[1];
        write_single_data(sensor_id_copy[0]);
    }
    else if (sensor_id_copy2_zero==1)
    {
        sensor_id_copy2[0]=sensor_id[0];
        sensor_id_copy2[1]=sensor_id[1];
        write_single_data1(sensor_id_copy2[0]);
    }

    Batt_MeasLevel();
    Batt_GetParameter(BATT_PARAM_LEVEL,&batt_level);

    //led blink depending on battery level
    if(batt_level>66)
    {
        for (i=0;i<3;i++)
        {
            PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_ON);
            DELAY_MS(100);
            PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_OFF);
            DELAY_MS(100);
        }
    }
    else if(batt_level<=66)
    {
        for (i=0;i<3;i++)
        {
            PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
            DELAY_MS(100);
            PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
            DELAY_MS(100);
        }
    }

    bspI2cInit();

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  IOCPortConfigureSet(IOID_13, IOC_PORT_RFC_GPO0, IOC_IOMODE_NORMAL); //** This command used to turn on the range extender **//
    // Map RFC_GPO1 to DIO6
  IOCPortConfigureSet(IOID_6, IOC_PORT_RFC_GPO1, IOC_IOMODE_NORMAL); //** This command used to turn on the range extender **//

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&buffer_clock, multi_role_clockHandler,
                 SEND_PERIOD, 0, false, SEND_BUFFER_DATA);
  Util_constructClock(&ack_clock, multi_role_clockHandler,
                 ACK_PERIOD, 0, false, ACK_TIMEOUT_EVT);
  Util_constructClock(&batt_clock, multi_role_clockHandler,
                 BATT_PERIOD, 0, false, BATT_EVT);
  Util_constructClock(&dyn_id_resend_clock, multi_role_clockHandler,
                 DYN_ID_RESEND_PERIOD, 0, false, DYNAMIC_ID_RESEND_EVT);
  Util_constructClock(&read_delay_clock, multi_role_clockHandler,
                 READ_DELAY_PERIOD, 0, false, 0);
  Util_constructClock(&tempstreamClock, multi_role_clockHandler,
                 TEMP_STREAM_PERIOD, 0, false, TEMP_STREAM_EVT);

  //Tx power set
  if(power_set_plus_5==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }
  else if(power_set_zero==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }
  else if(power_set_minus_6==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_6_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }
  else if(power_set_minus_12==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_12_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }
  else if(power_set_minus_18==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_18_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }
  else if(power_set_minus_21==1)
  {
      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_21_DBM);
      power_set_minus_21=0;
      power_set_minus_18=0;
      power_set_minus_12=0;
      power_set_minus_6=0;
      power_set_zero=0;
      power_set_plus_5=0;
  }

  reset_count++;
  //id and reset store
  write_single_data1(reset_count);
  write_single_data1(sensor_id_copy2[0]);

  count_restore();

  // Setup the GAP
  {
    // Set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);

    // Set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

    // Scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);

    // Set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, 8);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);

    // Register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }

  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = TRUE;
    uint16_t advertOffTime = 0;

    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);

    // Set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData, NULL);

    // Set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

    // set max amount of scan responses
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    // Set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                         &scanRes, NULL);

    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);

    // Allocate memory for index to connection handle map
    if (connHandleMap = ICall_malloc(sizeof(connHandleMapEntry_t) * maxNumBleConns))
    {
      // Init index to connection handle map
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        connHandleMap[i].connHandle = INVALID_CONNHANDLE;
      }
    }

    // Allocate memory for per connection discovery information
    if (discInfo = ICall_malloc(sizeof(discInfo_t) * maxNumBleConns))
    {
      // Init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        discInfo[i].charHdl = 0;
        discInfo[i].discState = BLE_DISC_STATE_IDLE;
        discInfo[i].svcEndHdl = 0;
        discInfo[i].svcStartHdl = 0;
      }
    }
  }

  // Open the OAD module and add the OAD service to the application
  if(OAD_SUCCESS != OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
  {
      /*
       *  OAD cannot be opened, steps must be taken in the application to
       *  handle this gracefully, this can mean an error, assert,
       *  or print statement.
       */
  }
  else
  {
      // Register the OAD callback with the application
      OAD_register(&multi_role_oadCBs);
  }

//  WatchdogApp_Init();

  // GATT
  {
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
    Batt_AddService();                           // Battery Service

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);
  }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  val=0x01;
  sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL3_C,&val,1);
  Sensor_init();

  //clock start
  Util_startClock(&batt_clock);
  Util_startClock(&buffer_clock);
  Util_startClock(&tempstreamClock);
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();
  UTC_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

//   Watchdog_clear(WatchdogHandle);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // OAD events
      if(events & OAD_OUT_OF_MEM_EVT)
      {
        // The OAD module is unable to allocate memory, cancel OAD
        OAD_cancel();
      }

      if(events & OAD_QUEUE_EVT)
      {
          // Process the OAD Message Queue
          uint8_t status = OAD_processQueue();

          // If the OAD state machine encountered an error, print it
          // Return codes can be found in oad_constants.h
          if(status == OAD_DL_COMPLETE)
          {
              // Report status
          }
          else if(status == OAD_IMG_ID_TIMEOUT)
          {
              // This may be an attack, terminate the link
              GAPRole_TerminateConnection(OAD_getactiveCxnHandle());
          }
          else if(status != OAD_SUCCESS)
          {
              // Report Error
          }
      }

      if(events & OAD_DL_COMPLETE_EVT)
      {
          // Register for L2CAP Flow Control Events
          L2CAP_RegisterFlowCtrlTask(selfEntity);
      }

      // If RTOS queue is not empty, process app message.
      if (events & MR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            multi_role_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      if(oad_in_progress==0)
      {
          if (events & TEMP_STREAM_EVT)
          {
              //sensor event
              if(TEMP_STREAM_PERIOD)
                  Util_startClock(&tempstreamClock);
              if (reset_before<100)
                  reset_before++;

              if(sensor_id_16!=0 && mesh_id!=0)
              {
                  if(time_received==1)
                  {
                      time_updation();
                      adv_update=1;
                      coin_adv();
                  }

                  //temp set
                  if(temp_enable==1)
                  {
                      temp_hum_loop++;
                      if(temp_hum_loop>=60)
                      {
                          temp_activity();
                          temp_hum_loop=0;
                      }
                  }

                  //acc and gyro set
                  if(acc_enable==1 || gyro_enable==1)
                  {
                      if(acc_start==1)
                          Activity();

                      if(read_rpt_start>0)
                      {
                          read_min_inc++;
                          if(read_min_inc>=read_time_insec)//before it was 10,cofigurabledata datarate min=5sec,max=ff(4.15min), (0,default)=10sec
                          {
                              read_5min++;
                              read_min_inc=0;
                          }
                      }

                      if(read_5min>=1) // depents avobe time control the sensor data read speed
                      {
                          read_5min=0;
                          read_min_inc=0;
                          n=0;
                          sensorLSM6DS3_Setinterrupt();
                          read_rpt_start=0;
                          acc_start=1;
                      }
                  }
              }
          }

          //send data loop
          if (events & SEND_BUFFER_DATA)
          {
              if (SEND_PERIOD)
                  gateway_count++;
                  Correction_count++;
                  Util_startClock(&buffer_clock);

               if(sensor_id_16!=0 && mesh_id!=0)
              {
                  //fm version type
                  if(firmw_notifyer==1)
                  {
                      acceData8[0]=sensor_id[0];
                      acceData8[1]=sensor_id[1];
                      acceData8[2]=firmware_use_case;
                      acceData8[3]=date;
                      acceData8[4]=month;
                      acceData8[5]=0;
                      memcpy(dataToSend[6],acceData8,SENSOR_LEN);
                      memset(acceData8,0,sizeof(acceData8));
                      firmw_notifyer=2;
                      add_to_buffer(6);
                  }

                  if(device_connected==0 && dyn_scan_start==1 && reset_before>30)
                      Connect_to_coin();
                  else if ((device_connected==0 && dynamic_id_16!=0x0064) && (buffer_counter>0 || buffer_counter2>0) && reset_before>30)
                      Connect_to_coin();

                  if(device_connected==1)
                  {
                      device_connected_inc++;

                      if(device_connected_inc>=80)
                      {
                          disconnect_and_update_buffer(ack_packet[1]);
                          dev_disconnect_central=1;
                      }
                  }
              }
 /*************************************************************/
             if((scanned_dyn_id[0]==0x01)|| (G_off==0x01))
             {
                     if((gateway_count<=30) && (dynamic_id_16!=0x0064))
                     {
                         turn_on_adv();
                         coin_adv();

                      }

                     if((device_connected==0) && (gateway_count>30 && gateway_count<=60) && reset_before>30)
                     {

                         adv1=FALSE;
                         GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv1, NULL);
                         Connect_to_coin();

                     }
                     if(gateway_count>61)
                         gateway_count=0;
             }

 /*************************************************************/
          }

          //ack loop
          if (events & ACK_TIMEOUT_EVT)
          {
              one_sec_count++;
                          if (one_sec_count>=60)
                          {
                              one_min_count++;
                              one_sec_count=0;
                          }
                            if (one_min_count>=70)
                             // one_min_count=0;

              if (status_resend==1 && one_sec_count>=30)
                   {
                          status_resend=0;
                         dummy_sensor_data[0]=sensor_id[0];
                         dummy_sensor_data[1]=sensor_id[1];
                         dummy_sensor_data[2]=0x11;
                         dummy_sensor_data[3]=222;
                         dummy_sensor_data[4]=0;
                         dummy_sensor_data[5]=0;

                          memcpy(dataToSend[1],dummy_sensor_data,SENSOR_LEN);
                          memset(dummy_sensor_data,0,sizeof(dummy_sensor_data));
                         one_sec_count=0;
                         count=0;
                         add_to_buffer(1);
                    }
              if(gatdata==0)
              {
                  disconnect_and_update_buffer(ack_packet[1]);
                  dev_disconnect_central=1;
              }
              else if(gatdata==1)
              {
                  disconnect_and_update_gateway_buffer(ack_packet[1]);
              }
         }

          //batt loop
          if (events & BATT_EVT)
          {
              if (BATT_PERIOD)
                  Util_startClock(&batt_clock);
             count1++;
             if(count1==2)
             {
               dynamic_id_resend(); // dynamic ID resend every 20 min //
               count1=0;
              }
              if(sensor_id_16!=0 && mesh_id!=0)
              {
                 //// batt_activity();
                  settings_changed=1;
                  count++;

                  if(count>=4)
                  {
                      dummy_sensor_data[0]=sensor_id[0];
                      dummy_sensor_data[1]=sensor_id[1];
                      dummy_sensor_data[2]=0x11;
                      dummy_sensor_data[3]=222;
                      dummy_sensor_data[4]=0;
                      dummy_sensor_data[5]=0;

                      memcpy(dataToSend[1],dummy_sensor_data,SENSOR_LEN);
                      memset(dummy_sensor_data,0,sizeof(dummy_sensor_data));
                      status_resend=1;
                      count=0;
                      add_to_buffer(1);
                  }
                    batt_init++;
                   if(batt_init>60)  batt_init=13;

                   if(batt_init<=12 || batt_init%6==0)
                       batt_activity();

                   if(batt_init==9 || batt_init==12 ||  batt_init==60)  /// in first one and halfhour and two hour it send battry data  after thean it will send every 8 hour
                       {
                           batt_data[0]=sensor_id[0];
                           batt_data[1]=sensor_id[1];
                           batt_data[2]=0x13;
                           batt_data[3]=bat_val;
                           batt_data[4]=0;
                           batt_data[5]=0;
                           memcpy(dataToSend[1],batt_data,SENSOR_LEN);
                           memset(batt_data,0,sizeof(batt_data));

                           add_to_buffer(1);

                       }
              }
          }

          //dyn id loop
          if (events & DYNAMIC_ID_RESEND_EVT)
          {
              if (DYN_ID_RESEND_PERIOD)
                  Util_startClock(&dyn_id_resend_clock);

              //dynamic_id_resend();
          }
      }
    }
  }
}

//acc and gyro loop
void Activity()
{
    bspI2cRelease();
    bspI2cAcquire(BSP_I2C_INTERFACE_0,SENSOR_I2C_ADDRESS);
    uint8_t dataToWrite = 0,val;

    if(freq_factor==0x10||freq_factor==0x20||freq_factor==0x30||freq_factor==0x40)
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
    else if(freq_factor==0x50)
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    else if(freq_factor==0x60)
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    else
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;

    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
    dataToWrite |= freq_factor;

    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL1_XL, &dataToWrite,1);
    sensorReadReg(LSM6DS3_ACC_GYRO_CTRL4_C,&dataToWrite,1);
    dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

    val=0x0E;
    sensorWriteReg(LSM6DS3_ACC_GYRO_TAP_CFG1, &val,1);
    val=0x01;
    sensorWriteReg(LSM6DS3_ACC_GYRO_TAP_THS_6D, &val,1);
    val=0x7F;
    sensorWriteReg(LSM6DS3_ACC_GYRO_INT_DUR2, &val,1);
    val=0x8F;//0F
    sensorWriteReg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, &val,1);
    val=0xE8;
    sensorWriteReg(LSM6DS3_ACC_GYRO_MD1_CFG, &val,1);
    val=0x60; //70
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL4_C, &val,1);

    if(n<5)// control no of data in single iteration
        sensorLSM6DS3_Setinterrupt();
    else if(n>=5) // stop data reading
        sensorLSM6DS3_Stopinterrupt();

    if(sensorReadScheduled==true && n<5 && !Util_isActive(&read_delay_clock))
    {
        Util_startClock(&read_delay_clock);
            if(read_rpt_start==0)
                read_rpt_start=1;

        sensorReadScheduled=false;

        if(acc_enable==1 && gyro_enable==2)
        {
            Accl_read();

            if(g>g1_low && g<g1_high)
            {
                PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_ON);
                acceData1[0]=sensor_id[0];
                acceData1[1]=sensor_id[1];
                acceData1[2]=0x1;
                acceData1[3]=10*g;
                acceData1[4]=0;
                acceData1[5]=0;
                memcpy(dataToSend[1],acceData1,SENSOR_LEN);
                memset(acceData1,0,sizeof(acceData1));

                add_to_buffer(1);
                n++;
            }
            else if(g>g2_low)
            {
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                acceData2[0]=sensor_id[0];
                acceData2[1]=sensor_id[1];
                acceData2[2]=0x2;
                acceData2[3]=10*g;
                if(acceData2[3]>=160)
                    acceData2[3]=160;
                acceData2[4]=0;
                acceData2[5]=0;
                memcpy(dataToSend[1],acceData2,SENSOR_LEN);
                memset(acceData2,0,sizeof(acceData2));

                add_to_buffer(1);
                n++;
            }
        }
        else if(acc_enable==2 && gyro_enable==1)
        {
            read_gyro();

            if(((ax_g_mod>g_low && ax_g_mod<g_high)||(ay_g_mod>g_low && ay_g_mod<g_high)||(az_g_mod>g_low && az_g_mod<g_high)) && g1>g_low && g1<g_high)//(ax_g>g3_x && ay_g<g3_y)
            {
                PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_ON);
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                acceData3[0]=sensor_id[0];
                acceData3[1]=sensor_id[1];
                acceData3[2]=0x3;
                acceData3[3]=0.1*g1;
                if(acceData3[3]>=0 && acceData3[3]<=1)
                    acceData3[3]=1;
                acceData3[4]=0;
                acceData3[5]=0;
                memcpy(dataToSend[1],acceData3,SENSOR_LEN);
                memset(acceData3,0,sizeof(acceData3));

                add_to_buffer(1);
                n++;
            }
            else if((ax_g_mod>g_high||ay_g_mod>g_high||az_g_mod>g_high) && g1>g_high)//(ax_g>g3_x1 && az_g<g3_z)
            {
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                PIN_setOutputValue(hGpioPin, Board_GLED,Board_LED_ON);
                acceData4[0]=sensor_id[0];
                acceData4[1]=sensor_id[1];
                acceData4[2]=0x4;
                acceData4[3]=0.1*g1;
                if(acceData4[3]>=200)
                    acceData4[3]=200;
                acceData4[4]=0;
                acceData4[5]=0;
                memcpy(dataToSend[1],acceData4,SENSOR_LEN);
                memset(acceData4,0,sizeof(acceData4));

                add_to_buffer(1);
                n++;
            }
        }
        else if(acc_enable==1 && gyro_enable==1)
        {
            Accl_read();
            read_gyro();

            if(g>g1_low && g<g1_high)
            {
                PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_ON);
                acceData1[0]=sensor_id[0];
                acceData1[1]=sensor_id[1];
                acceData1[2]=0x1;
                acceData1[3]=10*g;
                acceData1[4]=0;
                acceData1[5]=0;
                memcpy(dataToSend[1],acceData1,SENSOR_LEN);
                memset(acceData1,0,sizeof(acceData1));

                add_to_buffer(1);
                n++;
            }
            else if(g>g2_low)
            {
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                acceData2[0]=sensor_id[0];
                acceData2[1]=sensor_id[1];
                acceData2[2]=0x2;
                acceData2[3]=10*g;
                if(acceData2[3]>=160)
                    acceData2[3]=160;
                acceData2[4]=0;
                acceData2[5]=0;
                memcpy(dataToSend[1],acceData2,SENSOR_LEN);
                memset(acceData2,0,sizeof(acceData2));

                add_to_buffer(1);
                n++;
            }
            else if(((ax_g_mod>g_low && ax_g_mod<g_high)||(ay_g_mod>g_low && ay_g_mod<g_high)||(az_g_mod>g_low && az_g_mod<g_high)) && g1>g_low && g1<g_high)//(ax_g>g3_x && ay_g<g3_y)
            {
                PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_ON);
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                acceData3[0]=sensor_id[0];
                acceData3[1]=sensor_id[1];
                acceData3[2]=0x3;
                acceData3[3]=0.1*g1;
                if(acceData3[3]>=0 && acceData3[3]<=1)
                    acceData3[3]=1;
                acceData3[4]=0;
                acceData3[5]=0;
                memcpy(dataToSend[1],acceData3,SENSOR_LEN);
                memset(acceData3,0,sizeof(acceData3));

                add_to_buffer(1);
                n++;
            }
            else if((ax_g_mod>g_high||ay_g_mod>g_high||az_g_mod>g_high) && g1>g_high)//(ax_g>g3_x1 && az_g<g3_z)
            {
                PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                PIN_setOutputValue(hGpioPin, Board_GLED,Board_LED_ON);
                acceData4[0]=sensor_id[0];
                acceData4[1]=sensor_id[1];
                acceData4[2]=0x4;
                acceData4[3]=0.1*g1;
                if(acceData4[3]>=200)
                    acceData4[3]=200;
                acceData4[4]=0;
                acceData4[5]=0;
                memcpy(dataToSend[1],acceData4,SENSOR_LEN);
                memset(acceData4,0,sizeof(acceData4));

                add_to_buffer(1);
                n++;
            }
        }
        DELAY_MS(200);
        PIN_setOutputValue(hGpioPin, Board_BLED,Board_LED_OFF);
        PIN_setOutputValue(hGpioPin, Board_GLED,Board_LED_OFF);
        PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
    }

    val=0x0f;
    sensorWriteReg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, &val,1);
    val=0x00;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL2_G, &val,1);
    val=0x60; //70
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL4_C, &val,1);
    acc_start=0;
}

//acc read from registers values
void Accl_read(void)
{
    val=0x38;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL9_XL, &val,1);

    val=0x04;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL3_C,&val,1);

    val=0x10;//00
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL6_G, &val,1);

    if(freq_factor==0x10||freq_factor==0x20||freq_factor==0x30||freq_factor==0x40)
        val |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
    else if(freq_factor==0x50)
        val |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    else if(freq_factor==0x60)
        val |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    else
        val |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;

    val |= LSM6DS3_ACC_GYRO_FS_XL_16g;
    val |= freq_factor;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL1_XL, &val,1);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTX_L_XL,&ax_L,1);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTX_H_XL,&ax_H,1);

    ax_val=(int16) ( (ax_H<<8) | ax_L);

    acc_x1= dataconvert(ax_val);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTY_L_XL,&ay_L,1);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTY_H_XL,&ay_H,1);

    ay_val=(int16) ( (ay_H<<8) | ay_L);

    acc_y1= dataconvert(ay_val);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTZ_L_XL,&az_L,1);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTZ_H_XL,&az_H,1);

    az_val=(int16) ( (az_H<<8) | az_L);

    acc_z1= dataconvert(az_val);

    ax=(acc_x1*0.000488);
    g_x=ax;

    ay=(acc_y1*0.000488);
    g_y=ay;

    az=(acc_z1*0.000488);
    g_z=az;
    g=sqrt(pow(g_x,2)+pow(g_y,2)+pow(g_z,2));
}

//gyro read from register values
void read_gyro(void)
{
    val=0x8F;
    sensorWriteReg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, &val,1);

    val=0x20;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL4_C, &val,1);

    val=0x38;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL10_C, &val,1);

    val=0x80;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL7_G, &val,1);

    val=0x3C;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL2_G, &val,1);

    val=0x44;
    sensorWriteReg(LSM6DS3_ACC_GYRO_CTRL3_C,&val,1);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTX_L_G,&gx_L,1);
    sensorReadReg(LSM6DS3_ACC_GYRO_OUTX_H_G,&gx_H,1);

    gx_val=(int16) ( (gx_H << 8) | gx_L);
    gx_val1= dataconvert(gx_val);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTY_L_G,&gy_L,1);
    sensorReadReg(LSM6DS3_ACC_GYRO_OUTY_H_G,&gy_H,1);

    gy_val=(int16) ( (gy_H << 8) | gy_L);
    gy_val1= dataconvert(gy_val);

    sensorReadReg(LSM6DS3_ACC_GYRO_OUTZ_L_G,&gz_L,1);
    sensorReadReg(LSM6DS3_ACC_GYRO_OUTZ_H_G,&gz_H,1);

    gz_val=(int16) ( (gz_H << 8) | gz_L);
    gz_val1= dataconvert(gz_val);

    ax_g=gx_val1*0.070;
    ay_g=gy_val1*0.070;
    az_g=gz_val1*0.070;

    ax_g_mod=abs(ax_g);
    ay_g_mod=abs(ay_g);
    az_g_mod=abs(az_g);

    g1=sqrt(pow(ax_g_mod,2)+pow(ay_g_mod,2)+pow(az_g_mod,2));
}

//lsm6ds3 interrupt callback reg
void Sensor_init(void)
{
    sensorReadScheduled = false;

    if(sensorLSM6DS3Init())
        sensorLSM6DS3RegisterCallback(SensorLSM6DS3_processInterrupt);
}

void SensorLSM6DS3_processInterrupt()
{
    acc_start=1;
    sensorReadScheduled = true;
}

//acc register data conversion to decimal float values
static int16_t dataconvert(int16_t acc_data)
{
    static int16_t x_data,d[4],val[4],bval[15],b[15];
    uint8_t binsize = 15;
    uint8_t size = 4,i;

    x_data=acc_data;
    if ( (acc_data & 0x8000)==0x8000)
    {
        i=0;
        x_data = (~ x_data)+1;
        while(binsize != 0)
        {
            b[i] = ( x_data&0x1);
            i++;
            x_data =  x_data>> 1;
            binsize--;
        }

        sum = 0;
        for(i=0;i<15;i++)
        {
            bval[i] = b[i]*pow(2,i);
        }

        for(i=0;i<15;i++)
        {
            sum = sum+bval[i];
        }
        return(-1*sum);
    }
    else if ((acc_data& 0x8000)!=0x8000)
    {
        i=0;
        while(size != 0)
        {
            d[i] = ( x_data&0xF);
            i++;
            x_data =  x_data >> 4;
            size--;
        }

        sum = 0;
        val[0] = d[0];
        val[1] = d[1]*16;
        val[2] = d[2]*256;
        val[3] = d[3]*4096;

        for(i=0;i<4;i++)
        {
            sum = sum+val[i];
        }

        return(sum);
    }
    memset(bval,0,sizeof(bval));
    memset(b,0,sizeof(b));
}

//read temp and threshold check loop
void temp_activity()
{
    int tempdata=0;
    bspI2cRelease();
    bspI2cAcquire(BSP_I2C_INTERFACE_0,SENSOR_I2C_ADDRESS1);
    value=0xE3;
    bspI2cWriteSingle(value);
    DELAY_MS(14);
    bspI2cRead(data, 2);
    temperature=(((data[0] * 256 + data[1]) * 175.72) / 65536.0) - 46.85;

    if(temperature==0)
        neg_temp=126;
    else if(temperature<0 && temperature>-41)
        neg_temp=126-(temperature);
    else if(temperature>0 && temperature<126)
        neg_temp=temperature;

    temp_float=(neg_temp*100);
    temp_float1=temp_float % 100;

    if(temperature>Temp_threshold_high)
    {
        if(temp_high==0)
        {
            tempdata1[0]=sensor_id[0];
            tempdata1[1]=sensor_id[1];
            tempdata1[2]=0x6;
            tempdata1[3]=neg_temp;
            tempdata1[4]=temp_float1;
            tempdata1[5]=0;
            memcpy(dataToSend[1],tempdata1,SENSOR_LEN);
            memset(tempdata1,0,sizeof(tempdata1));
            init_temp_high=temperature;
            tempdata=1;
            temp_high=1;
        }
        if(abs(temperature-init_temp_high)>2)
            temp_high=0;
    }
    else if(temperature<Temp_threshold_low)
    {
        if(temp_low==0)
        {
            tempdata2[0]=sensor_id[0];
            tempdata2[1]=sensor_id[1];
            tempdata2[2]=0x5;
            tempdata2[3]=neg_temp;
            tempdata2[4]=temp_float1;
            tempdata2[5]=0;
            memcpy(dataToSend[1],tempdata2,SENSOR_LEN);
            memset(tempdata2,0,sizeof(tempdata2));
            init_temp_low=temperature;
            tempdata=1;
            temp_low=1;
        }
        if(abs(temperature-init_temp_low)>2)
            temp_low=0;
    }
    if(tempdata==1 && (temp_high==1 || temp_low==1))
    {
        add_to_buffer(1);
        tempdata=0;
    }
}

//data transfer fn
void DataSend(uint8* dataToSend,uint8 size,uint8_t chandle)
{
    attPrepareWriteReq_t req;
    req.pValue = GATT_bm_alloc(connHandleMap[cindex].connHandle, ATT_WRITE_REQ, size, NULL);

    if ( req.pValue != NULL )
    {
        req.handle = chandle;

        req.offset = 0;

        req.len = size;

        memcpy(req.pValue, dataToSend,size);
        status = GATT_WriteLongCharValue(connHandleMap[cindex].connHandle, &req, selfEntity);

        if ( status != SUCCESS )
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
}

//time updation fn
void time_updation()
{
    time_total=UTC_getClock();
    if(time_total>=86400)
    {
        UTC_setClock(0);
        time_hr=0;
        time_min=0;
        time_sec=0;

        if(input_date<=last_date)
            input_date=input_date+1;

        if(input_date>last_date)
            input_date=1;

        settings_changed=1;
    }
    else
    {
        time_hr=time_total/3600;
        time_total_dup=time_total%3600;
        time_min=time_total_dup/60;
        time_sec=time_total_dup%60;
    }
}

//utc clock start
void time_start()
{
    utc_time_server_input=(time_hr*3600)+(time_min*60)+time_sec;
    UTC_setClock(utc_time_server_input);
    time_received=1;
}

//time store fn
void time_save()
{
    time_date[0]=time_hr;
    time_date[1]=time_min;
    time_date[2]=time_sec;
    time_date[3]=input_date;
    time_date[4]=last_date;

    write_set_values(time_date);
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {
  #if !defined (USE_LL_CONN_PARAM_UPDATE)

              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
  #endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case GAP_MSG_EVENT:
      multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      break;

    case L2CAP_SIGNAL_EVENT:
    {
       // Process L2CAP free buffer notification
       multi_role_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
       break;
    }

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if( multi_role_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
      OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
  }

  // Messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    // Find index from connection handle
    connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      multi_role_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);

      // We're done with the response message
      multi_role_freeAttRsp(status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {

    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;

  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_CONN_EVT_END_EVT:
      {
        if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
        {
          // The GATT server might have returned a blePending as it was trying
          // to process an ATT Response. Now that we finished with this
          // connection event, let's try sending any remaining ATT Responses
          // on the next connection event.
          // Try to retransmit pending ATT Response (if any)
          multi_role_sendAttRsp();
        }

        if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_OAD_SEND))
        {
          // Wait until all pending messages are sent
          if(numPendingMsgs == 0)
          {
            // Reset the system
            HAL_SYSTEM_RESET();
          }
          numPendingMsgs--;

        }

          ICall_free(pMsg->pData);
          break;
      }

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Multi GAPRole event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_paramUpdateDecisionCB
*
* @brief   Callback for application to decide whether or not to accept
*          a parameter update request and, if accepted, what parameters
*          to use
*
* @param   pReq - pointer to param update request
* @param   pRsp - pointer to param update response
*
* @return  none
*/
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp)
{
  // Make some decision based on desired parameters. Here is an example
  // where only parameter update requests with 0 slave latency are accepted
  if (pReq->connLatency == 0)
  {
    // Accept and respond with remote's desired parameters
    pRsp->accepted = TRUE;
    pRsp->connLatency = pReq->connLatency;
    pRsp->connTimeout = pReq->connTimeout;
    pRsp->intervalMax = pReq->intervalMax;
    pRsp->intervalMin = pReq->intervalMin;
  }

  // Don't accept param update requests with slave latency other than 0
  else
  {
    pRsp->accepted = FALSE;
  }
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
    case GAP_DEVICE_INIT_DONE_EVENT:
    {

    }
    break;

    // Advertising started
    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // Advertising ended
    case GAP_END_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // A discovered device report
    case GAP_DEVICE_INFO_EVENT:
    {
        //adv data copy and compare in prcoess adv packet fn
        memset(device_name,0,sizeof(device_name));
        memcpy(adv_data,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen);

        ptr=&adv_data[2];
        memcpy(device_name,ptr,device_length);

        memcpy(curr_addr,pEvent->deviceInfo.addr,GATEWAY_DATA);
        devices_discovered++;

        if (devices_discovered!=1)
        {
            res=compare_string(curr_addr,prev_addr,GATEWAY_DATA);
            if (res==0)
            {
                memcpy(prev_addr,curr_addr,GATEWAY_DATA);
                dev_counter++;

                if(pEvent->deviceInfo.eventType==GAP_ADRPT_SCAN_RSP)
                {
                    if(compare_string(device_name,moving_device_name,moving_device_length)==1)
                        RSSI=pEvent->deviceInfo.rssi;
                    else if(compare_string(device_name,gateway_name,device_length)==1)
                        RSSI=pEvent->deviceInfo.rssi;

                    process_adv_packet();
                }
            }
            else if(res==1 && pEvent->deviceInfo.eventType==GAP_ADRPT_SCAN_RSP)
            {
                if(compare_string(device_name,moving_device_name,moving_device_length)==1)
                    RSSI=pEvent->deviceInfo.rssi;
                else if(compare_string(device_name,gateway_name,device_length)==1)
                    RSSI=pEvent->deviceInfo.rssi;

                process_adv_packet();
            }
        }
        else
        {
            memcpy(prev_addr,curr_addr,GATEWAY_DATA);
            dev_counter++;
        }
    }
    break;

    // End of discovery report
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
        uint8 dev_index;
        // Discovery complete
        scanningStarted = FALSE;
        scan_count++;

        if(scan_count==250)
        {
            scan_count=26;
            if (no_of_scan_cycle<10)
                no_of_scan_cycle++;
        }

        //rest dyn id if 25 scans
        if(scan_count>=25 && dyn_scan_start==0 && gatdata==0 )
        {
           dyn_scan_start=1;

            scan_count=0;
            range=3;
             //dynamic_id_16=0x0064;
             //dynamic_id[0]=0x00;
             //dynamic_id[1]=0x64;

            Util_stopClock(&dyn_id_resend_clock);
            Util_rescheduleClock(&dyn_id_resend_clock,DYN_ID_RESEND_PERIOD);
            //dynamic_resend=0;

            turn_on_adv();

            coin_adv();
        }

        if(same_device_transfer==2 && gatdata==0)
        {
            if((abs(dyn_id_previous_16-dynamic_id_16)>1) && (abs(dyn_id_previous_16-dynamic_id_16)!=0x0064))
            {
                same_device_transfer=0;
                dynamic_id_16=dyn_id_previous_16+1;
               // dynamic_id[0]=0x00;//(dynamic_id_16 & 0xFF00) >> 8;
                dynamic_id[1]=(dynamic_id_16 & 0x00FF);

                dynamic_data[0]=sensor_id[0];
                dynamic_data[1]=sensor_id[1];
                dynamic_data[2]=0x16;
                dynamic_data[3]=0x00;
                dynamic_data[4]=dynamic_id[1];
                dynamic_data[5]=0;
                if (last_dynamic_id != dynamic_id[1])
                {
                    last_dynamic_id=dynamic_id[1];
                    memcpy(dataToSend[5],dynamic_data,SENSOR_LEN);
                    add_to_buffer(5);
                }

//                memcpy(dataToSend[5],dynamic_data,SENSOR_LEN);
                memset(dynamic_data,0,sizeof(dynamic_data));

//                add_to_buffer(5);

                coin_adv();

                Util_startClock(&dyn_id_resend_clock);
            }
            else if(abs(dyn_id_previous_16-dynamic_id_16)==1)
            {
                same_device_transfer=0;
                memset(same_dev_address,0,sizeof(same_dev_address));
                dyn_id_previous_16=0;
            }
        }

        if(buffer_counter2!=0 && gatdata==1)
            turn_on_adv();

        // Copy all of the results
        scanRes = pEvent->discCmpl.numDevs;
        memcpy(devList, pEvent->discCmpl.pDevList, (sizeof(gapDevRec_t) * scanRes));

        //connection forming loop
        if (dev_list_index>0)
        {
            dest_index=find_dest_addr();

            if(dest_index!=0xFF && dyn_scan_start==0)
            {
                dev_index=scanned_coin_info[dest_index].DevList_index;
                memcpy(dest_addr,devList[dev_index].addr,GATEWAY_DATA);
                addrType1=devList[dev_index].addrType;

                if(same_device_transfer==0)
                    dyn_id_previous_16=scanned_coin_info[dest_index].dyn_id_16;

                //initialize scan index to last device
                GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,DEFAULT_LINK_WHITE_LIST,addrType1,dest_addr);
            }
            else if(dyn_scan_start==1 && dynamic_id_16!=0x0064)
            {
                dyn_scan_start=0;
                dev_list_index=0;
                scan_count=0;

                turn_on_adv();

                coin_adv();

                memset(scanned_coin_info,0,sizeof(scanned_coin_info));
                memset(scanned_dyn_id,0,sizeof(scanned_dyn_id));

                dynamic_data[0]=sensor_id[0];
                dynamic_data[1]=sensor_id[1];
                dynamic_data[2]=0x16;
                dynamic_data[3]=0x00;
                dynamic_data[4]=dynamic_id[1];
                dynamic_data[5]=0;
                if (last_dynamic_id != dynamic_id[1])
                {
                    last_dynamic_id=dynamic_id[1];
                    memcpy(dataToSend[5],dynamic_data,SENSOR_LEN);
                    add_to_buffer(5);
                }
//                 memcpy(dataToSend[5],dynamic_data,SENSOR_LEN);
                   memset(dynamic_data,0,sizeof(dynamic_data));

//                add_to_buffer(5);

                Util_startClock(&dyn_id_resend_clock);
            }
        }
    }
    break;

    // Connection has been established
    case GAP_LINK_ESTABLISHED_EVENT:
    {
      // If succesfully established
      if (pEvent->gap.hdr.status == SUCCESS)
      {
          device_connected=1;
          ix=0;jx=0,ig=0;

          PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_ON);

          // Add index-to-connHandle mapping entry and update menus
          cindex = multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.devAddr);

          memcpy(connected_device_address,pEvent->linkCmpl.devAddr,GATEWAY_DATA);

          connRole=pEvent->linkCmpl.connRole;

          //if central data transfer start
          if (connRole==GAP_PROFILE_CENTRAL)
          {
              if(same_device_transfer==0)
                  memcpy(same_dev_address,pEvent->linkCmpl.devAddr,GATEWAY_DATA);

              if(compare_string(connected_device_address,same_dev_address,GATEWAY_DATA)==1)
                  same_device_transfer++;
              else if(compare_string(connected_device_address,same_dev_address,GATEWAY_DATA)==0)
              {
                  same_device_transfer=0;
                  dyn_id_previous_16=0;
              }

              //one way coin to gateway
              if (buffer_counter!=0 && gatdata==0)
              {
                  read_from_flash(&coin_buffer[0],COIN_DATA);

                  if((coin_buffer[0]!=0x00 || coin_buffer[1]!=0x00) && coin_buffer[2]!=0x00)
                  {
                      ptr=&coin_buffer[0];
                      memcpy(packet_to_send,ptr,COIN_DATA);
                      DataSend(packet_to_send,COIN_DATA,simple1handle);
                  }
                  else
                  {
                      buff_limit_erase();

                      if(ext_addr_ch_bt==1)
                          ext_addr_ch_bt=2;
                      else if(ext_addr_ch_bt==2)
                          ext_addr_ch_bt=1;
                      write_single_data(ext_addr_ch_bt);

                      thresholddata[0]=sensor_id[0];
                      thresholddata[1]=sensor_id[1];
                      thresholddata[2]=0x45;
                      thresholddata[3]=0x4A;
                      thresholddata[4]=0x00;
                      thresholddata[5]=0x00;
                      thresholddata[6]=0x00;
                      gatewaydata();
                  }

                  if (status!=SUCCESS)
                  {
                      disconnect_and_update_buffer(ix);
                      dev_disconnect_central=1;
                  }
                  else if (status==SUCCESS)
                  {
                      Util_startClock(&ack_clock);
                  }
              }
              //reverse direction get data transfer
              else if(buffer_counter2!=0 && gatdata==1)
              {
                  ptr=&gateway_buffer[ig][0];
                  memcpy(packet_to_send1,ptr,GET_DATA);
                  DataSend(packet_to_send1,GET_DATA,simple3handle);
                  if (status!=SUCCESS)
                  {
                      disconnect_and_update_gateway_buffer(ig);
                  }
                  else if (status==SUCCESS)
                  {
                      Util_startClock(&ack_clock);
                  }
              }
          }
      }
    }
    break;

    // Connection has been terminated
    case GAP_LINK_TERMINATED_EVENT:
    {
      //adv on
      turn_on_adv();

      // read current num active so that this doesn't change before this event is processed
      uint8_t currentNumActive = linkDB_NumActive();

      // Find index from connection handle
      connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);

      // Cancel the OAD if one is going on
      // A disconnect forces the peer to re-identify
      OAD_cancel();

      // Check to prevent buffer overrun
      if (connIndex < maxNumBleConns)
      {
        // Clear screen, reset discovery info, and return to main menu
        connHandleMap[connIndex].connHandle = INVALID_CONNHANDLE;

        // Reset discovery info
        discInfo[connIndex].discState = BLE_DISC_STATE_IDLE;
        discInfo[connIndex].charHdl = 0;

        device_connected=0;
        device_connected_inc=0;
        if(dev_disconnect_central==0 && (gatdata==0 && connRole==GAP_PROFILE_CENTRAL))
        {
            disconnect_and_update_buffer(ack_packet[1]);
        }

        dev_disconnect_central=0;

        if(connRole==GAP_PROFILE_PERIPHERAL)
            memset(ack_packet,0,sizeof(ack_packet));

        PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_OFF);
        connRole=0;
      }
    }
    break;

    // A parameter update has occurred
    case GAP_LINK_PARAM_UPDATE_EVENT:
    {

    }
    break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  // Print new value depending on which characteristic was updated
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
      device_connected_inc=0;
      //config coin regsiter notify
      if(appflag==1)
      {
          DELAY_MS(500);
          SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t), &appdata);
          appflag=0;
      }

      //ack if peripheral
      if(sensor_id_16!=0 && mesh_id!=0 && connRole==GAP_PROFILE_PERIPHERAL)
      {
          ack_packet[0]=ack;
          ack_packet[1]=++jx;
          DataSend(ack_packet,ACK_DATA,simple1handle);

          if(compare_string(simplegattdata,last_coin_packet,COIN_DATA)==0)
          {
              if (status==SUCCESS)
              {
                  memcpy(dataToSend[2],simplegattdata,COIN_DATA);
                  add_to_buffer(2);
                  memcpy(last_coin_packet,simplegattdata,COIN_DATA);
                  memset(simplegattdata,0,sizeof(simplegattdata));
              }

              status=0xFF;
          }
      }
      //next data transfer if central
      else if (connRole==GAP_PROFILE_CENTRAL && sensor_id_16!=0 && mesh_id!=0) ////"" If the device is connected in central mode it starts sending data here **//
      {
          if (ack_packet[0]==ack && ack_packet[1]<buffer_counter && ack_packet[1]<250)
          {
              first_data_read=0;

              Util_stopClock(&ack_clock);
              read_from_flash(&coin_buffer[0],COIN_DATA);

              ptr=&coin_buffer[0];
              memcpy(packet_to_send,ptr,COIN_DATA);
              DataSend(packet_to_send,COIN_DATA,simple1handle);

              if (status!=SUCCESS)
              {
                  disconnect_and_update_buffer(ack_packet[1]);
                  dev_disconnect_central=1;
              }
              else
                  Util_startClock(&ack_clock);

              status=0xFF;
          }
          else if (ack_packet[0]==ack && ack_packet[1]==buffer_counter)
          {
              first_data_read=0;
              disconnect_and_update_buffer(ack_packet[1]);
              dev_disconnect_central=1;
          }
      }
    break;

  case SIMPLEPROFILE_CHAR3:
      gatdata=1;
      device_connected_inc=0;
      //cofigurabledatarate set from app
      if(coin_id[0]==0x55 && coin_id[2]==0x00 && coin_id[3]==0x00 && time_data_val==2)
      {
          time_data_val=0;
          read_time_insec= coin_id[1];
          if(read_time_insec<=5 && read_time_insec>0)
         {
           read_time_insec=5;
         }
          write_set_values(read_time_insec);
      }
     /* if(thresholddata[2]==03)
      {
          thresholddata[4]=
          setdata[0]=thresholddata[4]
      }*/
      //coin id set from app
      else if(((coin_id[0]!=0 || coin_id[1]!=0) && coin_id[2]!=0) && time_data_val==2)
      {
          if(coin_id[3]==01)
          {
              gateway_flg_en=1;
          }
          if(coin_id[3]==00)
          {
             gateway_flg_en=0;
          }
          time_data_val=0;
          buff_limit_erase();

          if (!isOpen)
          {
              isOpen = ExtFlash_open();
          }

          ExtFlash_erase(set_threshold_loc,1);

          if (isOpen)
          {
            isOpen = false;
            ExtFlash_close();
          }

          sensor_id[0]=coin_id[0];
          sensor_id[1]=coin_id[1];
          mesh_id=coin_id[2];

          sensor_id_copy[0]=coin_id[0];
          sensor_id_copy[1]=coin_id[1];
          mesh_id_copy=coin_id[2];

          sensor_id_copy2[0]=coin_id[0];
          sensor_id_copy2[1]=coin_id[1];
          mesh_id_copy2=coin_id[2];

          write_single_data(sensor_id[0]);
          write_single_data(mesh_id);

          write_single_data(sensor_id_copy[0]);
          write_single_data(mesh_id_copy);

          write_single_data1(sensor_id_copy2[0]);
          write_single_data1(mesh_id_copy2);

          memset(coin_id,0,sizeof(coin_id));

          flashread(); //here it reads the all settings
          sensor_id_16=sensor_id[0];
          sensor_id_16=(sensor_id_16<<8)|sensor_id[1];
          adv_up_ios=1;
          coin_adv();

          gatdata=0;

          //extflash new or old notify
          if(ext_fl_part==1)
          {
              DELAY_MS(500);
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t), &ext_id);
          }
          else if(ext_fl_part==0)
          {
              DELAY_MS(500);
              SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t), &ext_id1);
          }
      }
      else if(time_data_val==1)
      {
          //time set from app
          time_data_val=0;
          if(time_data[0]==sensor_id[0] && time_data[1]==sensor_id[1] && time_data[1]==0x42)
          {
              time_hr=time_data[3];
              time_min=time_data[4];
              time_sec=time_data[5];
              input_date=time_data[6];
              last_date=time_data[7];

              time_start();
              settings_changed=1;
              memset(time_data,0,sizeof(time_data));

              coin_adv();
          }
          gatdata=0;
      }
      else if(sensor_id_16!=0 && mesh_id!=0 && connRole==GAP_PROFILE_PERIPHERAL)
      {
          ack_packet[0]=ack;
          ack_packet[1]=++jx;
          DataSend(ack_packet,ACK_DATA,simple3handle);
          if(compare_string(thresholddata,last_coin_packet1,GET_DATA)==0)
          {
              if (status==SUCCESS)
              {
                  gatewaydata();
                  memcpy(last_coin_packet1,thresholddata,GET_DATA);
              }
          }
      }
      //get/set next data transfer
      else if (connRole==GAP_PROFILE_CENTRAL && sensor_id_16!=0 && mesh_id!=0)
      {
          if (ack_packet[0]==ack && ack_packet[1]<buffer_counter2)
          {
              Util_stopClock(&ack_clock);
              ptr=&gateway_buffer[ack_packet[1]][0];
              memcpy(packet_to_send1,ptr,GET_DATA);
              DataSend(packet_to_send1,GET_DATA,simple3handle);

              if (status!=SUCCESS)
                  disconnect_and_update_gateway_buffer(ack_packet[1]);
              else
                  Util_startClock(&ack_clock);

              status=0xFF;
          }
          else if (ack_packet[0]==ack && ack_packet[1]==buffer_counter2)
              disconnect_and_update_gateway_buffer(ack_packet[1]);
      }
    break;

  default:
    // Should not reach here!
    break;
  }
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // Allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // If sucessfully allocated
  if (pMsg)
  {
    // Fill up message
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
* @fn      multi_role_mapConnHandleToIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
* @return  index or INVALID_CONNHANDLE if connHandle isn't found
*/
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  // Loop through connection
  for (index = 0; index < maxNumBleConns; index ++)
  {
    // If matching connection handle found
    if (connHandleMap[index].connHandle == connHandle)
    {
      return index;
    }
  }
  // Not found if we got here
  return INVALID_CONNHANDLE;
}

/*********************************************************************
 * @fn      multi_role_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 */
static void multi_role_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      multi_role_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void multi_role_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if( multi_role_enqueueMsg(MR_CONN_EVT_END_EVT, (uint8_t *)pReport) == FALSE)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
* @fn      multi_role_addMappingEntry
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr)
{
  uint16_t index;
  // Loop though connections
  for (index = 0; index < maxNumBleConns; index++)
  {
    // If there is an open connection
    if (connHandleMap[index].connHandle == INVALID_CONNHANDLE)
    {
      // Store mapping
      connHandleMap[index].connHandle = connHandle;

      // Convert address to string
      uint8_t *pAddr = (uint8_t *) Util_convertBdAddr2Str(addr);

      // Copy converted string to persistent connection handle list
      memcpy(connHandleMap[index].strAddr, pAddr, B_STR_ADDR_LEN);

      return index;
    }
  }
  // No room if we get here
  return bleNoResources;
}

/*********************************************************************
*********************************************************************/

//add data to ext flash buffer
void add_to_buffer(uint8 dataToSend_no)
{
    uint8 d;
    if(packet_id==0xFF)
        packet_id=0;

    if(((dataToSend[dataToSend_no][0]!=0 || dataToSend[dataToSend_no][1]!=0) && dataToSend[dataToSend_no][2]!=0) &&
            ((dataToSend[dataToSend_no][0]!=0xFF || dataToSend[dataToSend_no][1]!=0xFF) && dataToSend[dataToSend_no][2]!=0xFF))
    {
        //if buff limit reach erase
        if((writecount>BUFFER_LIMIT && ext_fl_part==0) || (writecount>BUFFER_LIMIT1 && ext_fl_part==1))
        {
            if(ext_addr_ch_bt==1)
                ext_addr_ch_bt=2;
            else if(ext_addr_ch_bt==2)
                ext_addr_ch_bt=1;
            write_single_data(ext_addr_ch_bt);

            buff_limit_erase();
        }

        if(dataToSend_no!=2)
        {
            time_updation();
            dataToSend[dataToSend_no][COIN_DATA-5]=time_hr;
            dataToSend[dataToSend_no][COIN_DATA-4]=time_min;
            dataToSend[dataToSend_no][COIN_DATA-3]=time_sec;
            dataToSend[dataToSend_no][COIN_DATA-2]=input_date;
            dataToSend[dataToSend_no][COIN_DATA-1]=packet_id++;
            memcpy(dataToSend[0],dataToSend[dataToSend_no],COIN_DATA);

            for(d=0;d<COIN_DATA;d++)
                dataToSend[dataToSend_no][d]=0;
        }
        //data from other coin don't add time and packet id
        else if(dataToSend_no==2)
        {
            memcpy(dataToSend[0],dataToSend[dataToSend_no],COIN_DATA);

            for(d=0;d<COIN_DATA;d++)
                dataToSend[dataToSend_no][d]=0;
        }

        //write data in ext flash
        if(dataToSend[0][0]!=0 || dataToSend[0][1]!=0)
        {
            if (!isOpen)
            {
                isOpen = ExtFlash_open();
            }

            ptr=&dataToSend[0][0];
            if(ext_fl_part==0)
            {
                if(ext_addr_ch_bt==1)
                    writereturn = ExtFlash_write((extbuffer_loc0+(writecount*COIN_DATA)),COIN_DATA,ptr);
                else if(ext_addr_ch_bt==2)
                    writereturn = ExtFlash_write((extbuffer_loc1+(writecount*COIN_DATA)),COIN_DATA,ptr);
            }
            else if(ext_fl_part==1)
            {
                if(ext_addr_ch_bt==1)
                    writereturn = ExtFlash_write((extbuffer_loc2+(writecount*COIN_DATA)),COIN_DATA,ptr);
                else if(ext_addr_ch_bt==2)
                    writereturn = ExtFlash_write((extbuffer_loc3+(writecount*COIN_DATA)),COIN_DATA,ptr);
            }

            buffer_counter++;
            writecount++;
            if (isOpen)
            {
              isOpen = false;
              ExtFlash_close();
            }

            for(d=0;d<COIN_DATA;d++)
                dataToSend[0][d]=0;
        }

        if(device_connected==0 && buffer_ext_store==0)
        {
            if(buffer_counter==400 || buffer_counter==800 || buffer_counter==1200 || buffer_counter==1600 || buffer_counter==2200
                 || buffer_counter==3000 || buffer_counter==4000 || buffer_counter==5000 || buffer_counter==6000 || buffer_counter==7000 || buffer_counter==8000
                 || buffer_counter==9000 || buffer_counter==10000 || buffer_counter==11000 || buffer_counter==12000 || buffer_counter==13000 || buffer_counter==14000
                 || buffer_counter==15000 || buffer_counter==16000 || buffer_counter==17000 || buffer_counter==18000 || buffer_counter==19000 || buffer_counter==20000
                 || buffer_counter==21000 || buffer_counter==22000 || buffer_counter==23000 || buffer_counter==24000 || buffer_counter==25000 || buffer_counter==26000
                 || buffer_counter==28000 || buffer_counter==29000 || buffer_counter==30000 || buffer_counter==31000 || buffer_counter==32000 || buffer_counter==33000
                 || buffer_counter==34000 || buffer_counter==35000 || buffer_counter==36000 || buffer_counter==37000 || buffer_counter==38000 || buffer_counter==39000)
            {
                buff_store=1;
                count_store();
            }
        }
    }
}

//read sensor data from ext flash
void read_from_flash(uint8_t *buf,int buf_size)
{
    if(readcount<writecount && first_data_read==0)
    {
        if (!isOpen)
        {
            isOpen = ExtFlash_open();
        }

        if(ext_fl_part==0)
        {
            if(ext_addr_ch_bt==1)
                readreturn = ExtFlash_read((extbuffer_loc0+(readcount*buf_size)),buf_size,buf);
            else if(ext_addr_ch_bt==2)
                readreturn = ExtFlash_read((extbuffer_loc1+(readcount*buf_size)),buf_size,buf);
        }
        else if(ext_fl_part==1)
        {
            if(ext_addr_ch_bt==1)
                readreturn = ExtFlash_read((extbuffer_loc2+(readcount*buf_size)),buf_size,buf);
            else if(ext_addr_ch_bt==2)
                readreturn = ExtFlash_read((extbuffer_loc3+(readcount*buf_size)),buf_size,buf);
        }

        readcount++;
        data_read_count++;
        first_data_read=1;

        if (isOpen)
        {
          isOpen = false;
          ExtFlash_close();
        }
    }
    else if(readcount>writecount && buffer_counter<writecount)
    {
        readcount=writecount-buffer_counter;
    }
}

//buffer count store
void count_store(void)
{
    write_store[0] = writecount / 1000;
    write_mid=writecount % 1000;
    write_store[1] = write_mid / 100;
    write_store[2] = write_mid % 100;

    read_store[0] = readcount / 1000;
    read_mid=readcount % 1000;
    read_store[1] = read_mid / 100;
    read_store[2] = read_mid % 100;

    DELAY_MS(100);
    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }
    ExtFlash_erase(write_count_store,1);

    ptr = &write_store[0];
    writeretrn1=ExtFlash_write(write_count_store,3,ptr);

    ptr = &read_store[0];
    writeretrn1=ExtFlash_write(read_count_store,3,ptr);

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
}

//buffer count restore
void count_restore()
{
    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }

    ptr=&write_store[0];
    readreturn1=ExtFlash_read(write_count_store,3,ptr);
    if(write_store[0]==0xFF)
        memset(write_store,0,sizeof(write_store));

    ptr=&read_store[0];
    readreturn1=ExtFlash_read(read_count_store,3,ptr);
    if(read_store[0]==0xFF)
        memset(read_store,0,sizeof(read_store));

    if(write_store[0]!=0xFF && read_store[0]!=0xFF)
    {
        writecount = (write_store[0]*1000) + (write_store[1]*100) + write_store[2];
        readcount = (read_store[0]*1000) + (read_store[1]*100) + read_store[2];

        if(writecount > readcount)
        {
            buffer_counter=writecount-readcount;
            buffer_ext_store=1;
        }
    }

    if(buffer_counter<=2 || ((ext_fl_part==0 && buffer_counter>=BUFFER_LIMIT) || (ext_fl_part==1 && buffer_counter>=BUFFER_LIMIT1)))
    {
        buff_limit_erase();
    }

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
}

//erase extflash addressess
void buff_limit_erase()
{
    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }
    external_flash_erase();
    ExtFlash_erase(write_count_store,1);

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }

    buff_store=0;
    memset(write_store,0,sizeof(write_store));
    memset(read_store,0,sizeof(read_store));
    writecount=0;
    buffer_counter=0;
    readcount=0;
    first_data_read=0;
    buffer_ext_store=0;
}

void external_flash_erase()
{
    if(ext_fl_part==0)
    {
        if(ext_addr_ch_bt==1)
        {
            for(ij=0;ij<8;ij++)
            {
                DELAY_MS(5);
                ExtFlash_erase(extbuffer_loc(ij),1);
            }
        }
        else if(ext_addr_ch_bt==2)
        {
            for(ij=0;ij<8;ij++)
            {
                DELAY_MS(5);
                ExtFlash_erase(extbuffer_loc_1(ij),1);
            }
        }
    }
    else if(ext_fl_part==1)
    {
        if(ext_addr_ch_bt==1)
        {
            for(ij=0;ij<110;ij++)
            {
                DELAY_MS(5);
                ExtFlash_erase(extbuffer_loc_2(ij),1);
            }
        }
        else if(ext_addr_ch_bt==2)
        {
            for(ij=0;ij<110;ij++)
            {
                DELAY_MS(5);
                ExtFlash_erase(extbuffer_loc_3(ij),1);
            }
        }
    }
}

//coin scan start
void Connect_to_coin()
{
    uint8_t adv,i;
    dev_list_index=0;
    scanned_dyn_id_16=0;
    memset(scanned_coin_info,0,sizeof(scanned_coin_info));
    memset(scanned_dyn_id,0,sizeof(scanned_dyn_id));
    memset(adv_data,0,sizeof(adv_data));
    dev_counter=0;
    devices_discovered=0;

    if(gatdata==1)
    {
        if(buffer_counter>0 && buffer_counter2>=0)
            gatdata=0;
        else if(buffer_counter==0 && buffer_counter2>0)
            gatdata=1;
    }

    for (i=0;i<DEFAULT_MAX_SCAN_RES;i++)
    {
        devList[i].addrType=0;
        devList[i].eventType=0;
        memset(devList[i].addr,0,GATEWAY_DATA);
    }

    adv=FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);

    // Start or stop discovery
    if (linkDB_NumActive() < maxNumBleConns) //if we can connect to another device
    {
       if (!scanningStarted) //if we're not already scanning connecting_state
       {
           scanningStarted = TRUE;
           scanRes = 0;
           GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
       }
       else //cancel scanning
       {
           GAPRole_CancelDiscovery();
           scanningStarted = FALSE;
       }
    }
}

//compare coin or gateway after scan
void process_adv_packet()
{


    if(gatdata==0)
    {
 /////////////////////coin found///////////////////////////////////////////////////////////////////////////////////////////////////
    if (compare_string(device_name,moving_device_name,moving_device_length)==1) //coin
    {

    sensor_id_index=11;
    coin_scan_count++;

    if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+11]==mesh_id)
    {

      if((RSSI > RSSI_LIMIT) && ((adv_data[sensor_id_index+4])!=0x01))
       {

        //coin_scan_count++;
        valid_coin++;
        scanned_dyn_id[0]=adv_data[sensor_id_index+4];
        scanned_dyn_id[1]=adv_data[sensor_id_index+5];
       // scanned_dyn_id_16= scanned_dyn_id[0];
        scanned_dyn_id_16= scanned_dyn_id[1];//(scanned_dyn_id_16<<8) |
        //2<3 descending order



       if((scanned_dyn_id_16<dynamic_id_16) && (scanned_dyn_id[0]!=0x01 || G_off!=0x01 ))
        {
            scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
            scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
            scanned_coin_info[dev_list_index].RSSI=RSSI;
            scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
            memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
            scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;


             dev_list_index++;
             //coin_scan_count=0;
              G_off=0x00;
              j=0;

            if(time_received==0 || (time_hr!=0 || time_min!=0))
            {
                if(time_hr==adv_data[sensor_id_index+6] && time_min<adv_data[sensor_id_index+7])
                {
                    min_diff=(adv_data[sensor_id_index+7]-time_min);
                    min_diff=abs(min_diff);
                }

                if(((time_hr==adv_data[sensor_id_index+6] && (adv_data[sensor_id_index+7]>=time_min && min_diff>1))
                        || time_received==0) || time_hr!=adv_data[sensor_id_index+6])
                {
                    time_hr=adv_data[sensor_id_index+6];
                    time_min=adv_data[sensor_id_index+7];
                    time_sec=adv_data[sensor_id_index+8];

                    time_start();
                    settings_changed=1;
                }
            }

            if(input_date!=adv_data[sensor_id_index+9] && adv_data[sensor_id_index+9]!=0)
            {
                input_date=adv_data[sensor_id_index+9];
                settings_changed=1;
            }

            if(last_date!=adv_data[sensor_id_index+10] && adv_data[sensor_id_index+10]!=0)
            {
                last_date=adv_data[sensor_id_index+10];
                settings_changed=1;
            }
        }

       else if ((((dynamic_id_16!=0x01) && (scanned_dyn_id_16>=dynamic_id_16) && (coin_scan_count>160) && (buffer_counter>0)&&( scanned_dyn_id[0]!=0x01)&&(G_off!=0x01))||
                 ((dynamic_id_16==0x01) && (gateway_flg_en!=1) && (coin_scan_count>160)&& (buffer_counter>0) &&(G_off!=0x01))) ||
                 ((dynamic_id_16==0x01) &&((adv_data[sensor_id_index+4])!=0x01)&&(no_of_scan_cycle==5))) //&&(G_off!=0x01)
                {
                  dynamic_id_16=scanned_dyn_id_16+1;
                  coin_scan_count=0;//600=5min
                  no_of_scan_cycle=0;
                  G_off=0x00;
                }
            }
/************************************************/
      else if(((adv_data[sensor_id_index+4])==0x01) && (G_off!=0x01) && (coin_scan_count>120) && ((adv_data[sensor_id_index+5])<dynamic_id_16))

       {
          G_off=0x01;

          coin_scan_count=0;
          if(j<=3)
         {
         PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
         DELAY_MS(500);
         PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
         DELAY_MS(500);
         j++;
         }
       }
        else if  ((scanned_dyn_id[0]==0x00) && (coin_scan_count>120) && (G_off==0x01) && (scanned_dyn_id_16<dynamic_id_16))

       {
          dynamic_id_16=scanned_dyn_id_16+1;
          G_off=0x00;
          coin_scan_count=0;
       }
       else if ((no_of_scan_cycle==5)&&(buffer_counter>0) &&(G_off!=0x01))
       {

           if(j<=3)
            {
           PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
           DELAY_MS(1000);
           PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
           DELAY_MS(1000);
           j++;
            }
        }

    /*   else if((dynamic_id_16==1)&&(scanned_dyn_id_16==1)&&(coin_scan_count>160))

      {

         G_off=0x00;
         coin_scan_count=0;
      }*/

/************************************************/
   }
}

/////////////////////// if not coin check if gateway////////////////////////////////////////////////////////////////////////////////
       if (compare_string(device_name,gateway_name,device_length)==1)
        {
          //  gateway_scan_count++;
            sensor_id_index=9;
            if(adv_data[sensor_id_index]==9 && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+2]==0xFF
                    && adv_data[sensor_id_index+3]==0xFF && adv_data[sensor_id_index+9]==mesh_id)
            {


                if(RSSI > RSSI_LIMIT_1)
                {
                scanned_coin_info[dev_list_index].static_id[0]=0;
                scanned_coin_info[dev_list_index].static_id[1]=0;
                scanned_coin_info[dev_list_index].RSSI=RSSI;
                scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                scanned_coin_info[dev_list_index].dyn_id[0]=0;
                scanned_coin_info[dev_list_index].dyn_id[1]=0;
                scanned_coin_info[dev_list_index].dyn_id_16=0;
                scanned_coin_info[dev_list_index].in_range=TRUE;
                gateway_found=1;
                no_of_scan_cycle=0;
                G_off=0x00;
                j=0;
                if(buffer_counter>0)
                    dev_list_index++;


                if(time_received==0 || (time_hr!=0 || time_min!=0))
                {
                    if(time_hr==adv_data[sensor_id_index+4] && time_min<adv_data[sensor_id_index+5])
                    {
                        min_diff=(adv_data[sensor_id_index+5]-time_min);
                        min_diff=abs(min_diff);
                    }

                    if(((time_hr==adv_data[sensor_id_index+4] && (adv_data[sensor_id_index+5]>=time_min && min_diff>1))
                            || time_received==0) || time_hr!=adv_data[sensor_id_index+4])
                    {
                        time_hr=adv_data[sensor_id_index+4];
                        time_min=adv_data[sensor_id_index+5];
                        time_sec=adv_data[sensor_id_index+6];

                        time_start();
                        settings_changed=1;
                    }
                }

                if(input_date!=adv_data[sensor_id_index+7] && adv_data[sensor_id_index+7]!=0)
                {
                    input_date=adv_data[sensor_id_index+7];
                    settings_changed=1;
                }

                if(last_date!=adv_data[sensor_id_index+8] && adv_data[sensor_id_index+8]!=0)
                {
                    last_date=adv_data[sensor_id_index+8];
                    settings_changed=1;
                }
            }
        }
            /*************************************************************/
   }
        /*************************************************************/
 if((dynamic_id_16==1) && (no_of_scan_cycle==2)&& (gateway_flg_en==1))
                {
                    gateway_found=0;
                    G_off=0x01;
                    if(j<=3)
                    {
                    PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
                    DELAY_MS(1000);
                    PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
                    DELAY_MS(1000);
                    j++;
                    }
                }
        }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //get data coin scan loop, reverse direction scan
    else if(gatdata==1)
{
    if(compare_string(device_name,moving_device_name,moving_device_length)==1)
    {
        sensor_id_index=11;
        same_dyn_id=0;

        if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+2]==gateway_buffer[0][0] &&
                adv_data[sensor_id_index+3]==gateway_buffer[0][1] && adv_data[sensor_id_index+11]==mesh_id)
        {
            scanned_dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_dyn_id_16=scanned_dyn_id[0];
            scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

            scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
            scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
            scanned_coin_info[dev_list_index].RSSI=RSSI;
            scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
            memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
            scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

            if (RSSI > RSSI_LIMIT)
                {scanned_coin_info[dev_list_index].in_range=TRUE;}
            else if (RSSI < RSSI_LIMIT)
                {scanned_coin_info[dev_list_index].in_range=FALSE;}

            dev_list_index++;
        }
        else if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+4]==gateway_buffer[0][4] &&
                adv_data[sensor_id_index+5]==gateway_buffer[0][5] && adv_data[sensor_id_index+11]==mesh_id)
        {
            scanned_dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_dyn_id_16=scanned_dyn_id[0];
            scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

            scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
            scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
            scanned_coin_info[dev_list_index].RSSI=RSSI;
            scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
            memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
            scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

            if (RSSI >= RSSI_LIMIT)
                {scanned_coin_info[dev_list_index].in_range=TRUE;}
            else if(RSSI < RSSI_LIMIT)
                {scanned_coin_info[dev_list_index].in_range=FALSE;}

            dev_list_index++;
        }
        else if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+11]==mesh_id)
        {
            scanned_dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_dyn_id_16=scanned_dyn_id[0];
            scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

            if((scanned_dyn_id_16<getset_dynid_16) && (abs(scanned_dyn_id_16-dynamic_id_16)>0))
            {
                scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                scanned_coin_info[dev_list_index].RSSI=RSSI;
                scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                if (RSSI > RSSI_LIMIT)
                    {scanned_coin_info[dev_list_index].in_range=TRUE;}
                else if (RSSI < RSSI_LIMIT)
                { scanned_coin_info[dev_list_index].in_range=FALSE;}

                dev_list_index++;
            }
        }
        else if((adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8) && adv_data[sensor_id_index+11]==mesh_id)
        {
            scanned_dyn_id[0]=adv_data[sensor_id_index+4];
            scanned_dyn_id[1]=adv_data[sensor_id_index+5];
            scanned_dyn_id_16=scanned_dyn_id[0];
            scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

            if((scanned_dyn_id_16<getset_dynid_16) && scanned_dyn_id_16<0x64)
            {
                scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                scanned_coin_info[dev_list_index].RSSI=RSSI;
                scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                if (RSSI > RSSI_LIMIT)
                    {scanned_coin_info[dev_list_index].in_range=TRUE;}
                else if (RSSI < RSSI_LIMIT)
                    {scanned_coin_info[dev_list_index].in_range=FALSE;}

                dev_list_index++;
            }
        }
    }
  }
}

//compare if strings are equal
uint8 compare_string(uint8 *str1,uint8 *str2,uint8 size)
{
    uint8_t i,flag=1;
    for (i=0;i<size;i++)
    {
        if (*str1++ != *str2++)
        {
            flag=0;
            break;
        }
    }
    return flag;
}

//find address of coin to connect in list of scanned coins
uint8 find_dest_addr()
{
    uint8_t scenario=0,diff=0;
    uint8_t i;
    uint16_t dest_id;
    uint8_t dest_id_index;
    uint8_t devices_in_range=0,devices_out_range=0;
    int8 highest_RSSI = -96;
    int8 lowest_RSSI = -60;


    for (i=0;i<dev_list_index;i++)
    {
        if (scanned_coin_info[i].in_range == TRUE)
           devices_in_range++;
        else
           devices_out_range++;
    }

    for (i=0;i<dev_list_index;i++)
     {
         scan_dyn_list[i]=scanned_coin_info[i].dyn_id_16;
         valid_coin++;
      }


    if (devices_out_range==0 && devices_in_range != 0 )
        scenario = 1;
    else if (devices_out_range!=0 && devices_in_range == 0 )
        scenario = 2;
    else if (devices_out_range!=0 && devices_in_range != 0)
        scenario = 3;
    else
        scenario = 0xFF;

  if(dyn_scan_start==1)
    {
        dest_id=0xFFFF;

              for (i=0;i<dev_list_index;i++)
            {
              if ((scanned_coin_info[i].dyn_id_16<dest_id) && (dynamic_id_16!=0x0064)&& (scanned_coin_info[i].RSSI>=-90)&& (valid_coin>3)&&(G_off!=0x01))
                {
                  dest_id=scanned_coin_info[i].dyn_id_16;
                  diff=abs(dynamic_id_16-scanned_coin_info[i].dyn_id_16);

                          if (diff==1)
                              {
                                //dest_id=scanned_coin_info[i].dyn_id_16;
                                dynamic_id_16=scanned_coin_info[i].dyn_id_16+1;
                              }
                          valid_coin=0;
                 }

              else if ((dynamic_id_16==0x0064) && (G_off!=0x01))
                {
                  if((scanned_coin_info[i].dyn_id_16<dest_id) && (scanned_coin_info[i].RSSI>=-90)&& (valid_coin>3))
                  {
                     dest_id=scanned_coin_info[i].dyn_id_16;
                     dynamic_id_16=scanned_coin_info[i].dyn_id_16+1;
                     valid_coin=0;
                  }
                }
              }

            //dynamic_id[0]=0x00;//(dynamic_id_16 & 0xFF00) >> 8;
            dynamic_id[1]=(dynamic_id_16 & 0x00FF);
      return 0xFF;
    }
 else if(dyn_scan_start==0)
    {
        if(gatdata==0)
        {
            dest_id=0xFFFF;

                for (i=0;i<dev_list_index;i++)
                    {
                        if (scanned_coin_info[i].dyn_id_16<dynamic_id_16)
                               {

                                // dest_id=scanned_coin_info[i].dyn_id_16;
                                 dest_id_index=i;
                               }
                      }
            }

        else if(gatdata==1)
        {
            dest_id=0x0000;
            if (scenario!=0xFF)
            {
                if(scenario==1 || scenario==3)
                {
                    if(same_dyn_id==1)
                    {
                        for (i=0;i<dev_list_index;i++)
                        {
                            if((scanned_coin_info[i].in_range == TRUE) && (scanned_coin_info[i].RSSI < lowest_RSSI))
                            {
                                dest_id_index = i;
                                lowest_RSSI=scanned_coin_info[i].RSSI;
                            }
                        }
                    }
                    else if(same_dyn_id==0)
                    {
                        for (i=0;i<dev_list_index;i++)
                        {
                            if((scanned_coin_info[i].in_range == TRUE) && (scanned_coin_info[i].dyn_id_16 > dest_id))
                            {
                                dest_id_index = i;
                                dest_id=scanned_coin_info[i].dyn_id_16;
                            }
                        }
                    }
                }
                else if (scenario == 2)
                {
                    for (i=0;i<dev_list_index;i++)
                    {
                        if((scanned_coin_info[i].in_range == FALSE) && (scanned_coin_info[i].RSSI > highest_RSSI))
                        {
                            dest_id_index = i;
                            highest_RSSI=scanned_coin_info[i].RSSI;
                        }
                    }
                }
            }
        }

        return dest_id_index;
    }
    else
        return 0xFF;
}

//disconnect and clear buffer after data transfer between coin/gateway
void disconnect_and_update_buffer(uint8 next_index)//disconnects the buffer and updates the contents of the buffer
{
    uint8 s1;

    if(Util_isActive(&ack_clock))
        Util_stopClock(&ack_clock);

    dis_status=GAPRole_TerminateConnection(connHandleMap[cindex].connHandle);//GAP_CONNHANDLE_INIT

    scan_count=0;
    memset(ack_packet,0,sizeof(ack_packet));

    if (next_index!=0)
    {
        if(buffer_counter==next_index)
        {
            buffer_counter=writecount-readcount;

            if(buffer_counter==0 && buffer_counter2>0)
                gatdata=1;

            if(buffer_ext_store==1)
            {
                if(ext_addr_ch_bt==1)
                    ext_addr_ch_bt=2;
                else if(ext_addr_ch_bt==2)
                    ext_addr_ch_bt=1;
                write_single_data(ext_addr_ch_bt);

                buff_limit_erase();
            }

            if(buff_store==1)
            {
                DELAY_MS(100);
                buff_store=0;
                memset(write_store,0,sizeof(write_store));
                memset(read_store,0,sizeof(read_store));

                if (!isOpen)
                {
                    isOpen = ExtFlash_open();
                }

                ExtFlash_erase(write_count_store,1);

                if (isOpen)
                {
                  isOpen = false;
                  ExtFlash_close();
                }
            }

            if(writecount==readcount && ((writecount>=2650 && ext_fl_part==0) || (writecount>=35500 && ext_fl_part==1)))
            {
                if(ext_addr_ch_bt==1)
                    ext_addr_ch_bt=2;
                else if(ext_addr_ch_bt==2)
                    ext_addr_ch_bt=1;
                write_single_data(ext_addr_ch_bt);

                buff_limit_erase();
            }
        }
        else if (next_index<buffer_counter)
        {
            data_read_count=data_read_count-next_index;
            readcount=readcount-data_read_count;
            buffer_counter=writecount-readcount;
        }
     connRole=0;
    }
    else if(first_data_read==1 && readcount>0)
    {
        readcount=readcount-1;
    }

    if(settings_changed==1)
    {
        settings_changed=0;
        time_save();
    }

    dev_list_index=0;
    scanned_dyn_id_16=0;
    dev_counter=0;
    devices_discovered=0;
    data_read_count=0;
    memset(connected_device_address,0,sizeof(connected_device_address));
    memset(dest_addr,0,sizeof(dest_addr));
    memset(packet_to_send,0,sizeof(packet_to_send));
    memset(curr_addr,0,sizeof(curr_addr));
    memset(prev_addr,0,sizeof(prev_addr));
    memset(adv_data,0,sizeof(adv_data));
    memset(scanned_dyn_id,0,sizeof(scanned_dyn_id));

    for(s1=0;s1<DEV_LIST_SIZE;s1++)
    {
        scanned_coin_info[s1].DevList_index=0;
        scanned_coin_info[s1].RSSI=0;
        memset(scanned_coin_info[s1].addr1,0,GATEWAY_DATA);
        memset(scanned_coin_info[s1].dyn_id,0,2);
        memset(scanned_coin_info[s1].static_id,0,2);
        scanned_coin_info[s1].in_range=0;
        scanned_coin_info[s1].dyn_id_16=0;
    }

    for (s1=0;s1<DEFAULT_MAX_SCAN_RES;s1++)
    {
        devList[s1].addrType=0;
        devList[s1].eventType=0;
        memset(devList[s1].addr,0,GATEWAY_DATA);
    }

    first_data_read=0;
}

//disconnects the buffer and updates the contents of the buffer for rev communication
void disconnect_and_update_gateway_buffer(uint8 next_index1)//disconnects the buffer and updates the contents of the buffer
{
    uint8 s1;

    if(Util_isActive(&ack_clock))
        Util_stopClock(&ack_clock);

    GAPRole_TerminateConnection(connHandleMap[cindex].connHandle);

    scan_count=0;
    memset(ack_packet,0,sizeof(ack_packet));

    if (next_index1!=0)
    {
        if(buffer_counter2==next_index1)
        {
            memset(gateway_buffer,0,sizeof(gateway_buffer));
            buffer_counter2=0;

            gatdata=0;
        }
     connRole=0;
    }

    dev_list_index=0;
    scanned_dyn_id_16=0;
    dev_counter=0;
    devices_discovered=0;
    memset(connected_device_address,0,sizeof(connected_device_address));
    memset(dest_addr,0,sizeof(dest_addr));
    memset(packet_to_send1,0,sizeof(packet_to_send1));
    memset(curr_addr,0,sizeof(curr_addr));
    memset(prev_addr,0,sizeof(prev_addr));
    memset(adv_data,0,sizeof(adv_data));
    memset(scanned_dyn_id,0,sizeof(scanned_dyn_id));

    for(s1=0;s1<DEV_LIST_SIZE;s1++)
    {
        scanned_coin_info[s1].DevList_index=0;
        scanned_coin_info[s1].RSSI=0;
        memset(scanned_coin_info[s1].addr1,0,GATEWAY_DATA);
        memset(scanned_coin_info[s1].dyn_id,0,2);
        memset(scanned_coin_info[s1].static_id,0,2);
        scanned_coin_info[s1].in_range=0;
        scanned_coin_info[s1].dyn_id_16=0;
    }

    for (s1=0;s1<DEFAULT_MAX_SCAN_RES;s1++)
    {
        devList[s1].addrType=0;
        devList[s1].eventType=0;
        memset(devList[s1].addr,0,GATEWAY_DATA);
    }
}

//get/set fn for changing threshold,en/disable sensors etc
void gatewaydata()
{
    if (thresholddata[0]==sensor_id[0] && thresholddata[1]==sensor_id[1])
    {
        if(thresholddata[2]!=0x00 && thresholddata[3]==0x00 && thresholddata[4]==0x00)
        {
            switch(thresholddata[2])
            {
            case 1 :
                    if(acc_enable==1)
                    {
                        Accl_read();
                        thresholddata[3]=10*g;
                    }
                    set_get=1;
                    break;

            case 2 :
                    if(acc_enable==1)
                    {
                        Accl_read();
                        thresholddata[3]=10*g;
                    }
                    set_get=1;
                    break;

            case 3 :
                    if(gyro_enable==1)
                    {
                        read_gyro();

                        if(0.1*g1>=0 && 0.1*g1<=10)
                            thresholddata[3]=1;
                        else
                            thresholddata[3]=0.1*g1;
                    }
                    set_get=1;
                    break;

            case 4 :
                    if(gyro_enable==1)
                    {
                        read_gyro();

                        if(0.1*g1>=0 && 0.1*g1<=10)
                            thresholddata[3]=1;
                        else
                            thresholddata[3]=0.1*g1;
                    }
                    set_get=1;
                    break;

            case 5 :
                    if(temp_enable==1)
                    {
                        thresholddata[3]=neg_temp;
                        thresholddata[4]=temp_float1;
                    }
                    set_get=1;
                    break;

            case 6 :
                    if(temp_enable==1)
                    {
                        thresholddata[3]=neg_temp;
                        thresholddata[4]=temp_float1;
                    }
                    set_get=1;
                    break;

            case 19 :
                    Batt_MeasLevel();
                    Batt_GetParameter(BATT_PARAM_LEVEL,&batt_level);

                    if(batt_level>80 && batt_level<=100)
                        thresholddata[3]=1;
                    else if(batt_level>66 && batt_level<=80)
                        thresholddata[3]=2;
                    else if(batt_level<=66)
                        thresholddata[3]=3;
                    set_get=1;
                    break;

            default:
                    break;
            }

            if(set_get==1)
            {
                memcpy(dataToSend[9],thresholddata,COIN_DATA);
                add_to_buffer(9);
                set_get=0;
            }
        }
        else if(thresholddata[2]==0x18 && thresholddata[3]!=0x00 && thresholddata[4]==0x00)
        {
            switch(thresholddata[3])
            {
             case 1:
                 freq_factor=0x10;
                 set_get=1;
                 break;

             case 2:
                 freq_factor=0x20;
                 set_get=1;
                 break;

             case 3:
                 freq_factor=0x30;
                 set_get=1;
                 break;

             case 4:
                 freq_factor=0x40;
                 set_get=1;
                 break;

             case 5:
                 freq_factor=0x50;
                 set_get=1;
                 break;

             case 6:
                 freq_factor=0x60;
                 set_get=1;
                 break;

             case 7:
                 freq_factor=0x70;
                 set_get=1;
                 break;

             case 8:
                 freq_factor=0x80;
                 set_get=1;
                 break;

             case 9:
                 freq_factor=0x90;
                 set_get=1;
                 break;

             case 10:
                 freq_factor=0xA0;
                 set_get=1;
                 break;

             default:
                 break;
            }

            if(set_get==1)
            {
                thresholddata[4]=thresholddata[3];
                thresholddata[3]=thresholddata[2];
                thresholddata[2]=0x27;

                memcpy(dataToSend[9],thresholddata,COIN_DATA);
                add_to_buffer(9);

                write_single_data(freq_factor);
                set_get=0;
            }
        }
        else if (thresholddata[2]!=0x00 && thresholddata[3]==0x00 && thresholddata[4]!=0x00 && thresholddata[4]!=0xFF)
        {
            switch(thresholddata[2])
            {
             case 1 :
                     if(acc_enable==1)
                     {
                         if(thresholddata[4]==1)
                             g1_low=0.001;
                         else if(thresholddata[4]==2)
                             g1_low=0.1;
                         else
                             g1_low=thresholddata[4]*0.125;

                         g1_high=g1_low+2.375;
                     }
                     set_get=1;
                     break;

             case 2 :
                     if(acc_enable==1)
                     {
                         if(thresholddata[4]==1)
                             g2_low=0.001;
                         else if(thresholddata[4]==2)
                             g2_low=0.1;
                         else
                             g2_low=thresholddata[4]*0.125;
                     }
                     set_get=1;
                     break;

             case 3 :
                     if(gyro_enable==1)   //command 0009 03 00 01 00 00
                     {
                         check1=thresholddata[4];
                          if(check1>=0xCA && check1<=0xD1)
                          {
                              g_low=check1-0xC8;
                          }
                          else g_low=thresholddata[4]*10;
                     }
                     set_get=1;
                     break;

             case 4 :
                     if(gyro_enable==1)
                         g_high=thresholddata[4]*10;
                     set_get=1;
                     break;

             case 5 :
                     if(temp_enable==1)
                     {
                         if(thresholddata[4]==126)
                             Temp_threshold_low=0;
                         else if(thresholddata[4]>=127)
                             Temp_threshold_low=126-thresholddata[4];
                         else if(thresholddata[4]>0 && thresholddata[4]<=125)
                             Temp_threshold_low=thresholddata[4];

                         temp_low=0;
                     }
                     set_get=1;
                     break;

             case 6 :
                     if(temp_enable==1)
                     {
                         if(thresholddata[4]==126)
                             Temp_threshold_high=0;
                         else if(thresholddata[4]>=127)
                             Temp_threshold_high=126-thresholddata[4];
                         else if(thresholddata[4]>0 && thresholddata[4]<=125)
                             Temp_threshold_high=thresholddata[4];

                         temp_high=0;
                     }
                     set_get=1;
                     break;

             default:
                     break;
            }

            if(set_get==1)
            {
                if(g1_low>=0.001 && g1_low<0.1)
                    g1_low_ext=g1_low*1000;
                else if(g1_low>=0.1 && g1_low<=2.5)
                {
                    g1_low_ext=g1_low*100;
                    g1_low_ext_1=0;
                }
                else if(g1_low>=2.625 && g1_low<=16)
                {
                    g1_low_ext_1=g1_low*10;
                    g1_low_ext=0;
                }

                if(g2_low>=0.001 && g2_low<0.1)
                    g2_low_ext=g2_low*1000;
                else if(g2_low>=0.1 && g2_low<=2.5)
                {
                    g2_low_ext=g2_low*100;
                    g2_low_ext_2=0;
                }
                else if(g2_low>=2.625 && g2_low<=16)
                {
                    g2_low_ext_2=g2_low*10;
                    g2_low_ext=0;
                }

                if(g_low>=2 && g_low<=250)
                {
                    g_low_ext=g_low;
                    g_low_ext_1=0;
                }
                else if(g_low>=260)
                {
                    g_low_ext_1=g_low/10;
                    g_low_ext=0;
                }

                if(g_high>=10 && g_high<=250)
                {
                    g_high_ext=g_high;
                    g_high_ext_1=0;
                }
                else if(g_high>=260)
                {
                    g_high_ext_1=g_high/10;
                    g_high_ext=0;
                }

                if(Temp_threshold_low==0)
                    temp_thld_low=126;
                else if(Temp_threshold_low<0)
                    temp_thld_low=126-(Temp_threshold_low);
                else if(Temp_threshold_low>0)
                    temp_thld_low=Temp_threshold_low;

                if(Temp_threshold_high==0)
                    temp_thld_high=126;
                else if(Temp_threshold_high<0)
                    temp_thld_high=126-(Temp_threshold_high);
                else if(Temp_threshold_high>0)
                    temp_thld_high=Temp_threshold_high;

                setdata[0]=temp_thld_low;
                setdata[1]=temp_thld_high;
                setdata[2]=Humidity_threshold_low;
                setdata[3]=Humidity_threshold_high;
                setdata[4]=g1_low_ext;
                setdata[5]=g2_low_ext;
                setdata[6]=g1_low_ext_1;
                setdata[7]=g2_low_ext_2;
                setdata[8]=g_low_ext;
                setdata[9]=g_high_ext;
                setdata[10]=g_low_ext_1;
                setdata[11]=g_high_ext_1;

                thresholddata[3]=thresholddata[2];
                thresholddata[2]=0x27;
                memcpy(dataToSend[9],thresholddata,COIN_DATA);
                add_to_buffer(9);

                write_set_values(setdata);
                set_get=0;
            }
        }
        else if(thresholddata[2]==0x32 && thresholddata[3]!=0x00 && thresholddata[4]==0x00)
        {
            acceData8[0]=sensor_id[0];
            acceData8[1]=sensor_id[1];
            acceData8[2]=0x27;
            acceData8[3]=firmware_use_case;
            acceData8[4]=0;
            acceData8[5]=0;
            memcpy(dataToSend[9],acceData8,SENSOR_LEN);
            memset(acceData8,0,sizeof(acceData8));
            add_to_buffer(9);
        }
        else if(thresholddata[2]==0x45 && thresholddata[3]!=0x00 && thresholddata[4]==0x00)
        {
            tx_pwr_data=thresholddata[3];
            write_single_data(tx_pwr_data);

            HAL_SYSTEM_RESET();
        }
        else if(thresholddata[2]!=0x00 && thresholddata[3]!=0x00 && thresholddata[4]==0x00)
        {
            switch(thresholddata[2])
            {
             case 65 :
                      if(thresholddata[3]==0x45)
                      {
                          acc_enable=1;
                          acc_start=1;
                      }
                      else if(thresholddata[3]==0x44)
                      {
                          acc_enable=2;
                          if(gyro_enable==2)
                              acc_start=1;
                      }
                      set_get=1;
                      break;

             case 71 :
                      if(thresholddata[3]==0x45)
                      {
                          gyro_enable=1;
                          acc_start=1;
                      }
                      else if(thresholddata[3]==0x44)
                      {
                          gyro_enable=2;
                          if(acc_enable==2)
                              acc_start=1;
                      }
                      set_get=1;
                      break;

             case 84 :
                     if(thresholddata[3]==0x45)
                     {
                         temp_enable=1;
                     }
                     else if(thresholddata[3]==0x44)
                     {
                         temp_enable=2;
                     }
                     set_get=1;
                     break;

             default:
                     break;
            }

            if(set_get==1)
            {
                thresholddata[4]=thresholddata[3];
                thresholddata[3]=thresholddata[2];
                thresholddata[2]=0x27;

                memcpy(dataToSend[9],thresholddata,COIN_DATA);
                add_to_buffer(9);

                sensor_enable_data[0]=acc_enable;
                sensor_enable_data[1]=temp_enable;
                sensor_enable_data[2]=hum_enable;
                sensor_enable_data[3]=temp_stream_enable;
                sensor_enable_data[4]=hum_stream_enable;
                sensor_enable_data[5]=gyro_enable;

                write_set_values(sensor_enable_data);
                set_get=0;
            }
        }
        else if(thresholddata[2]==0xFF && thresholddata[3]==0x00 && thresholddata[4]==0xFF)
        {
            buff_limit_erase();

            if (!isOpen)
            {
                isOpen = ExtFlash_open();
            }

            ExtFlash_erase(set_threshold_loc,1);

            if (isOpen)
            {
              isOpen = false;
              ExtFlash_close();
            }
            flashread();
        }
        memset(thresholddata,0,sizeof(thresholddata));
        gatdata=0;
    }
    else
    {
        memcpy(gateway_buffer,thresholddata,COIN_DATA);
        memset(thresholddata,0,sizeof(thresholddata));
        buffer_counter2=1;
        getset_dynid_16=gateway_buffer[0][5];
        getset_dynid_16=(getset_dynid_16<<8) | gateway_buffer[0][6];
    }
}

//battery calculation fn
void batt_activity()
{
    Batt_MeasLevel();
    Batt_GetParameter(BATT_PARAM_LEVEL,&batt_level);
/*    batt_count++;

    if(batt_init==0)
    {
        batt_data[0]=sensor_id[0];
        batt_data[1]=sensor_id[1];
        batt_data[2]=0x13;

        if(batt_level>80 && batt_level<=100)
            batt_data[3]=1;
        else if(batt_level>66 && batt_level<=80)
            batt_data[3]=2;
        else if(batt_level<=66)
            batt_data[3]=3;

        batt_data[4]=0;
        batt_data[5]=0;
        batt_init=1;
        memcpy(dataToSend[1],batt_data,SENSOR_LEN);
        memset(batt_data,0,sizeof(batt_data));

        add_to_buffer(1);
    }

    if(batt_level>66 && batt_level<=80 && batt_85==0)
    {
        batt_data[0]=sensor_id[0];
        batt_data[1]=sensor_id[1];
        batt_data[2]=0x13;
        batt_data[3]=2;
        batt_data[4]=0;
        batt_data[5]=0;
        memcpy(dataToSend[1],batt_data,SENSOR_LEN);
        memset(batt_data,0,sizeof(batt_data));
        batt_85=1;

        add_to_buffer(1);
    }
    else if(batt_level<=66 && batt_75==0)
    {
        batt_data[0]=sensor_id[0];
        batt_data[1]=sensor_id[1];
        batt_data[2]=0x13;
        batt_data[3]=3;
        batt_data[4]=0;
        batt_data[5]=0;
        memcpy(dataToSend[1],batt_data,SENSOR_LEN);
        memset(batt_data,0,sizeof(batt_data));
        batt_75=1;

        add_to_buffer(1);
    }
    else if(batt_count>=24)
    {
        batt_count=0;
        batt_data[0]=sensor_id[0];
        batt_data[1]=sensor_id[1];
        batt_data[2]=0x13;

        if(batt_level>80 && batt_level<=100)
            batt_data[3]=1;
        else if(batt_level>66 && batt_level<=80)
            batt_data[3]=2;
        else if(batt_level<=66)
            batt_data[3]=3;

        batt_data[4]=0;
        batt_data[5]=0;
        memcpy(dataToSend[1],batt_data,SENSOR_LEN);
        memset(batt_data,0,sizeof(batt_data));

        add_to_buffer(1);
    }*/
  if (last_batt_level>batt_level)
             {
                 battry_drain_fast++;
                 last_batt_level=batt_level;

                 if(batt_level>80 && batt_level<=100)
                     batt_H++;
                 else if(batt_level>66 && batt_level<=80)
                     batt_M++;

                 else if(batt_level<=66)
                     batt_L++;

             }

             else if (last_batt_level<=batt_level)
             {
                 battry_same++;
                 last_batt_level=batt_level;
             }

  if(batt_init==9 || batt_init==12 || batt_init==60)
    {
       if (battry_drain_fast>=battry_same)
       {
         if(batt_L>2)
            bat_val=3;
         else if(batt_M>2)
           bat_val=2;
         else bat_val=1;
       }
      else
      {
       if(batt_level>80 && batt_level<=100)
          bat_val=1;
      else if(batt_level>66 && batt_level<=80)
           bat_val=2;
      else if(batt_level<=66)
           bat_val=3;
       }
          battry_drain_fast=0;
          battry_same=0;
          batt_H=0;
          batt_M=0;
          batt_L=0;

    }
}

//get ble address from registers
void GetAddress(void)
{
    memcpy(&BLEADDRESS[0],BLE0,sizeof(*BLE0));
    memcpy(&BLEADDRESS[1],BLE1,sizeof(*BLE1));
    memcpy(&BLEADDRESS[2],BLE2,sizeof(*BLE2));
    memcpy(&BLEADDRESS[3],BLE3,sizeof(*BLE3));
    memcpy(&BLEADDRESS[4],BLE4,sizeof(*BLE4));
    memcpy(&BLEADDRESS[5],BLE5,sizeof(*BLE5));
}

//coin adv change or set fn
void coin_adv()
{
    s=sensor_id_16/10000+0x30;
    x=sensor_id_16%10000;
    t=x/1000+0x30;
    y=x%1000;
    u=y/100+0x30;
    z1=y%100;
    v=z1/10+0x30;
    w=z1%10;
    w=w+0x30;

    attDeviceName[0]='C';
    attDeviceName[1]='M';
    attDeviceName[2]='P';
    attDeviceName[3]='F';
    attDeviceName[4]=s;
    attDeviceName[5]=t;
    attDeviceName[6]=u;
    attDeviceName[7]=v;
    attDeviceName[8]=w;

    scanRspData[6]=s;
    scanRspData[7]=t;
    scanRspData[8]=u;
    scanRspData[9]=v;
    scanRspData[10]=w;
    scanRspData[13]=sensor_id[0];
    scanRspData[14]=sensor_id[1];
    scanRspData[15]= G_off; //flag           ((gateway_flg_en)|(G_off>>8))
    scanRspData[16]=dynamic_id[1];
    scanRspData[17]=time_hr;
    scanRspData[18]=time_min;
    scanRspData[19]=time_sec;
    scanRspData[20]=input_date;
    scanRspData[21]=last_date;
    scanRspData[22]=mesh_id;
    scanRspData[23]=reset_count;
    //scanRspData[24]=reset_count;

    advertData[7]=BLEADDRESS[1];
    advertData[8]=BLEADDRESS[0];
    advertData[9]=BLEADDRESS[3];
    advertData[10]=BLEADDRESS[2];
    advertData[11]=BLEADDRESS[5];
    advertData[12]=BLEADDRESS[4];

    advertData[5]=sensor_id[1];
    advertData[6]=sensor_id[0];

    //scan rsp data
    if(adv_update==1)
    {
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
    }

    //advert data
    if(adv_up_ios==1)
    {
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);
        GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
        adv_up_ios=0;
    }
}

//adv on
void turn_on_adv()
{
    uint8_t advert = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advert, NULL);
}

void write_set_values(uint8_t* data_to_write_set)
{
    loc3=0;loc2=0,loc20=0,loc21=0,loc22=0;
    DELAY_MS(100);

    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }
    ExtFlash_erase(sensor_enable_loc,1);

    ptr=&sensor_enable_data[0];
    writereturn=ExtFlash_write(loc3+sensor_enable_loc,6,ptr);
    ptr=&read_time_insec;
    writereturn=ExtFlash_write(loc21+activity_read_time_loc,1,ptr);

    ptr=&setdata[0];
    writereturn=ExtFlash_write(loc2+set_threshold_loc,12,ptr);

    ptr=&time_date[0];
    writereturn=ExtFlash_write(loc20+time_date_loc,5,ptr);

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
}

void write_single_data(uint8_t single_data)
{
    loc16=0,loc6=0,loc=0,loc1=0,loc13=0,loc18=0,loc14=0,loc15=0,loc21=0,loc22=0;
    DELAY_MS(100);

    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }
    ExtFlash_erase(sensor_loc,1);

    ptr=&sensor_id[0];
    writereturn=ExtFlash_write(loc+sensor_loc,2,ptr);

    ptr=&tx_pwr_data;
    writereturn=ExtFlash_write(loc6+tx_power_loc,1,ptr);

    ptr=&freq_factor;
    writereturn=ExtFlash_write(loc16+freq_range_loc,1,ptr);

    ptr=&mesh_id;
    writereturn=ExtFlash_write(loc13+mesh_id_loc,1,ptr);

    ptr=&ext_addr_ch_bt;
    writereturn=ExtFlash_write(loc18+extflash_addr_choice,1,ptr);

    ExtFlash_erase(sensor_copy_loc1,1);
    ptr=&sensor_id_copy[0];
    writereturn=ExtFlash_write(loc1+sensor_copy_loc1,2,ptr);

    ptr=&mesh_id_copy;
    writereturn=ExtFlash_write(loc14+mesh_id_copy_loc1,1,ptr);

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
}

void write_single_data1(uint8_t single_data1)
{
    loc15=0,loc12=0,loc17=0;
    DELAY_MS(100);

    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }
    ExtFlash_erase(reset_count_loc,1);

    ptr=&reset_count;
    writereturn=ExtFlash_write(loc15+reset_count_loc,1,ptr);

    ptr=&sensor_id_copy2[0];
    writereturn=ExtFlash_write(loc12+sensor_copy_loc2,2,ptr);

    ptr=&mesh_id_copy2;
    writereturn=ExtFlash_write(loc17+mesh_copy_loc2,1,ptr);

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
}

//read all settings stored of coin
bool flashread()
{
    loc=0,loc1=0,loc2=0,loc3=0,loc4=0,loc5=0,loc6=0,loc7=0,loc8=0,loc12=0,loc13=0,loc14=0,loc15=0,loc16=0,loc17=0,loc18=0,loc20=0,loc21=0,loc22=0;

    if (!isOpen)
    {
        isOpen = ExtFlash_open();
    }

    ptr=&sensor_id[0];
    readreturn=ExtFlash_read(loc+sensor_loc,2,ptr);

    if ((sensor_id[0]==0xFF && sensor_id[1]==0xFF) || (sensor_id[0]==0x00 && sensor_id[1]==0x00))
    {
        sensor_id_zero=1;
        sensor_id[0]=0;
        sensor_id[1]=0;
    }

    ptr=&sensor_id_copy[0];
    readreturn=ExtFlash_read(loc1+sensor_copy_loc1,2,ptr);

    if ((sensor_id_copy[0]==0xFF && sensor_id_copy[1]==0xFF) || (sensor_id_copy[0]==0x00 && sensor_id_copy[1]==0x00))
    {
        sensor_id_copy_zero=1;
        sensor_id_copy[0]=0;
        sensor_id_copy[1]=0;
    }

    ptr=&sensor_id_copy2[0];
    readreturn=ExtFlash_read(loc12+sensor_copy_loc2,2,ptr);

    if ((sensor_id_copy2[0]==0xFF && sensor_id_copy2[1]==0xFF) || (sensor_id_copy2[0]==0x00 && sensor_id_copy2[1]==0x00))
    {
        sensor_id_copy2_zero=1;
        sensor_id_copy2[0]=0;
        sensor_id_copy2[1]=0;
    }

    if (sensor_id_zero==1 && sensor_id_copy_zero==1 && sensor_id_copy2_zero==1)
    {
        sensor_id[0]=0;
        sensor_id[1]=0;
        sensor_id_16=0;
    }
    else if (sensor_id_zero==1 && sensor_id_copy_zero==1 && sensor_id_copy2_zero==0)
    {
        sensor_id[0]=sensor_id_copy2[0];
        sensor_id[1]=sensor_id_copy2[1];
        sensor_id_16=sensor_id[0]<<8;
        sensor_id_16=sensor_id_16 | sensor_id[1];
       // write_single_data(sensor_id[0]);
    }
    else if (sensor_id_zero==1 && sensor_id_copy_zero==0 && sensor_id_copy2_zero==1)
    {
        sensor_id[0]=sensor_id_copy[0];
        sensor_id[1]=sensor_id_copy[1];
        sensor_id_16=sensor_id[0]<<8;
        sensor_id_16=sensor_id_16 | sensor_id[1];
      //  write_single_data(sensor_id[0]);
    }
    else if (sensor_id_zero==0 && sensor_id_copy_zero==1 && sensor_id_copy2_zero==1)
    {
        sensor_id_16=sensor_id[0]<<8;
        sensor_id_16=sensor_id_16 | sensor_id[1];
    }
    else if (sensor_id_zero==0 && sensor_id_copy_zero==0 && sensor_id_copy2_zero==0)
    {
        if ((sensor_id[0]==sensor_id_copy2[0] && sensor_id[1]==sensor_id_copy2[1]) || (sensor_id[0]==sensor_id_copy[0] && sensor_id[1]==sensor_id_copy[1]))
        {
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
        }
        if ((sensor_id_copy[0]==sensor_id_copy2[0] && sensor_id_copy[1]==sensor_id_copy2[1]) || (sensor_id[0]==sensor_id_copy[0] && sensor_id[1]==sensor_id_copy[1]))
        {
            sensor_id[0]=sensor_id_copy[0];
            sensor_id[1]=sensor_id_copy[1];
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
           // write_single_data(sensor_id[0]);
        }
        else
        {
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
        }
    }
    else if (sensor_id_zero==0 && (sensor_id_copy_zero==0 || sensor_id_copy2_zero==0))
    {
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
    }
    else if (sensor_id_copy_zero==0 && (sensor_id_zero==0 || sensor_id_copy2_zero==0))
    {
            sensor_id[0]=sensor_id_copy[0];
            sensor_id[1]=sensor_id_copy[1];
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
           // write_single_data(sensor_id[0]);
    }
    else if (sensor_id_copy2_zero==0 && (sensor_id_zero==0 || sensor_id_copy_zero==0))
    {
            sensor_id[0]=sensor_id_copy2[0];
            sensor_id[1]=sensor_id_copy2[1];
            sensor_id_16=sensor_id[0]<<8;
            sensor_id_16=sensor_id_16 | sensor_id[1];
           // write_single_data(sensor_id[0]);
    }
    else {
        sensor_id_16=sensor_id[0];
        sensor_id_16=(sensor_id_16<<8);
        sensor_id_16=(sensor_id_16 | sensor_id[1]);
    }
///////////////////

    ptr=&mesh_id;
    readreturn=ExtFlash_read(loc13+mesh_id_loc,1,ptr);

    if (mesh_id==0xFF)
        mesh_id=0;

    ptr=&mesh_id_copy;
    readreturn=ExtFlash_read(loc14+mesh_id_copy_loc1,1,ptr);

    if (mesh_id_copy==0xFF)
        mesh_id_copy=0;

    if(mesh_id!=mesh_id_copy)
    {
        if(mesh_id==0)
            mesh_id=mesh_id_copy;
        else if(mesh_id_copy==0)
            mesh_id_copy=mesh_id;
    }

    ptr=&mesh_id_copy2;
    readreturn=ExtFlash_read(loc17+mesh_copy_loc2,1,ptr);

    if (mesh_id_copy2==0xFF)
        mesh_id_copy2=0;

    if(mesh_id!=mesh_id_copy2)
    {
        if(mesh_id==0)
            mesh_id=mesh_id_copy2;
        else if(mesh_id_copy2==0)
            mesh_id_copy2=mesh_id;
    }
    ptr=&read_time_insec;
    readreturn=ExtFlash_read(loc21+activity_read_time_loc,1,ptr);
    if (read_time_insec==0xFF || read_time_insec==0x00)
        read_time_insec=0x0A;
    ptr=&reset_count;
    readreturn=ExtFlash_read(loc15+reset_count_loc,sizeof(uint8),ptr);
    if (reset_count==0xFF)
        reset_count=0;

    ptr=&setdata[0];
    readreturn=ExtFlash_read(loc2+set_threshold_loc,12,ptr);

    if((setdata[0]>0 && setdata[0]<=181) && (setdata[0]!=0xFF && setdata[1]!=0xFF))
    {
        if(setdata[0]==126)
            temp_thld_low=0;
        else if(setdata[0]>=127)
            temp_thld_low=126-setdata[0];
        else if(setdata[0]>0 && setdata[0]<=125)
            temp_thld_low=setdata[0];

        Temp_threshold_low=temp_thld_low;

        if(setdata[1]==126)
            temp_thld_high=0;
        else if(setdata[1]>=127)
            temp_thld_high=126-setdata[1];
        else if(setdata[1]>0 && setdata[1]<=125)
            temp_thld_high=setdata[1];

        Temp_threshold_high=temp_thld_high;

        Humidity_threshold_low=setdata[2];
        Humidity_threshold_high=setdata[3];

        if(setdata[4]==1)
            g1_low=0.001;
        else if(setdata[4]>=10 && setdata[4]<=250)
        {
            g1_low_ext=setdata[4];
            g1_low=g1_low_ext/100;
        }
        else if(setdata[6]>=26 && setdata[6]<=160)
        {
            g1_low_ext_1=setdata[6];
            g1_low=g1_low_ext_1/10;
        }

        g1_high=g1_low+2.375;

        if(setdata[5]==1)
            g2_low=0.001;
        else if(setdata[5]>=10 && setdata[5]<=250)
        {
            g2_low_ext=setdata[5];
            g2_low=g2_low_ext/100;
        }
        else if(setdata[7]>=26 && setdata[7]<=160)
        {
            g2_low_ext_2=setdata[7];
            g2_low=g2_low_ext_2/10;
        }

        if(setdata[8]>=10 && setdata[8]<=250)
        {
            g_low_ext=setdata[8];
            g_low=g_low_ext;
        }
        else if(setdata[10]>=26 && setdata[10]<=200)
        {
            g_low_ext_1=setdata[10]*10;
            g_low=g_low_ext_1;
        }

        if(setdata[9]>=10 && setdata[9]<=250)
        {
            g_high_ext=setdata[9];
            g_high=g_high_ext;
        }
        else if(setdata[11]>=26 && setdata[11]<=200)
        {
            g_high_ext_1=setdata[11]*10;
            g_high=g_high_ext_1;
        }
    }
    else
    {
        g1_low=1,g1_high=3.375,g2_low=3.375,g_low=2,g_high=1000;
        Temp_threshold_high=70,Temp_threshold_low=5;
        Humidity_threshold_high=95,Humidity_threshold_low=1;
    }

    ptr=&sensor_enable_data[0];
    readreturn=ExtFlash_read(loc3+sensor_enable_loc,6,ptr);

    if(sensor_enable_data[0]==0x01 || sensor_enable_data[0]==0x02)
    {
        acc_enable=sensor_enable_data[0];
        temp_enable=sensor_enable_data[1];
        hum_enable=sensor_enable_data[2];
        temp_stream_enable=sensor_enable_data[3];
        hum_stream_enable=sensor_enable_data[4];
        gyro_enable=sensor_enable_data[5];
    }
    else
    {
        acc_enable=1,temp_enable=1,hum_enable=1,temp_stream_enable=1,hum_stream_enable=1,gyro_enable=1;
    }

    ptr=&time_date[0];
    readreturn=ExtFlash_read(loc20+time_date_loc,5,ptr);

    if(time_date[0]!=0xFF && time_date[1]!=0xFF && time_date[2]!=0xFF && time_date[0]<=0x18)
    {
        time_hr=time_date[0];
        time_min=time_date[1];
        time_sec=time_date[2];
        input_date=time_date[3];
        last_date=time_date[4];
        time_start();
    }
    else
    {
        time_hr=0,time_min=0,time_sec=0,input_date=0,last_date=0;
    }

    if(last_date>31)
        last_date=30;

    if(input_date>31)
        input_date=1;

    ptr=&tx_pwr_data;
    readreturn=ExtFlash_read(loc6+tx_power_loc,1,ptr);

    if(tx_pwr_data==0xFF || tx_pwr_data==0x00)
    {
       power_set_minus_21=0;
       power_set_minus_18=0;
       power_set_minus_12=0;
       power_set_minus_6=0;
       power_set_zero=0;
       power_set_plus_5=1;
    }
    else if(tx_pwr_data==0x4A)
    {
       power_set_minus_21=0;
       power_set_minus_18=0;
       power_set_minus_12=0;
       power_set_minus_6=0;
       power_set_zero=0;
       power_set_plus_5=1;
    }
    else if(tx_pwr_data==0x4B)
    {
       power_set_minus_21=0;
       power_set_minus_18=0;
       power_set_minus_12=0;
       power_set_minus_6=0;
       power_set_zero=1;
       power_set_plus_5=0;
    }
    else if(tx_pwr_data==0x4C)
    {
       power_set_minus_21=0;
       power_set_minus_18=0;
       power_set_minus_12=0;
       power_set_minus_6=1;
       power_set_zero=0;
       power_set_plus_5=0;
    }
    else if(tx_pwr_data==0x4D)
    {
       power_set_minus_21=0;
       power_set_minus_18=0;
       power_set_minus_12=1;
       power_set_minus_6=0;
       power_set_zero=0;
       power_set_plus_5=0;
    }
    else if(tx_pwr_data==0x4E)
    {
       power_set_minus_21=0;
       power_set_minus_18=1;
       power_set_minus_12=0;
       power_set_minus_6=0;
       power_set_zero=0;
       power_set_plus_5=0;
    }
    else if(tx_pwr_data==0x4F)
    {
       power_set_minus_21=1;
       power_set_minus_18=0;
       power_set_minus_12=0;
       power_set_minus_6=0;
       power_set_zero=0;
       power_set_plus_5=0;
    }

    ptr=&freq_factor;
    readreturn=ExtFlash_read(loc16+freq_range_loc,1,ptr);

    if(freq_factor==0x10)
       freq_factor=0x10;
    else if(freq_factor==0x20)
       freq_factor=0x20;
    else if(freq_factor==0x30)
       freq_factor=0x30;
    else if(freq_factor==0x40)
       freq_factor=0x40;
    else if(freq_factor==0x50)
       freq_factor=0x50;
    else if(freq_factor==0x60)
       freq_factor=0x60;
    else if(freq_factor==0x70)
       freq_factor=0x70;
    else if(freq_factor==0x80)
       freq_factor=0x80;
    else if(freq_factor==0x90)
       freq_factor=0x90;
    else if(freq_factor==0xA0)
       freq_factor=0xA0;
    else
        freq_factor=0x60;

    ptr=&ext_addr_ch_bt;
    readreturn=ExtFlash_read(loc18+extflash_addr_choice,1,ptr);

    if(ext_addr_ch_bt==0x01)
        ext_addr_ch_bt=1;
    else if(ext_addr_ch_bt==0x02)
        ext_addr_ch_bt=2;
    else
        ext_addr_ch_bt=1;

    if (isOpen)
    {
      isOpen = false;
      ExtFlash_close();
    }
    return readreturn;
}

//dyn id resend fn after 5,30min
void dynamic_id_resend()
{
//    dynamic_resend++;
//if (dynamic_resend==1)
//{
    dynamic_data[0]=sensor_id[0];
    dynamic_data[1]=sensor_id[1];
    dynamic_data[2]=0x16;
    dynamic_data[3]=0x00;
    dynamic_data[4]=dynamic_id[1];
    dynamic_data[5]=0;
//}
    memcpy(dataToSend[5],dynamic_data,SENSOR_LEN);
    memset(dynamic_data,0,sizeof(dynamic_data));

        add_to_buffer(5);

//    if(dynamic_resend==1)
//    {
//        Util_stopClock(&dyn_id_resend_clock);
//        Util_rescheduleClock(&dyn_id_resend_clock,1799999);
//        Util_startClock(&dyn_id_resend_clock);
//    }
//    else if(dynamic_resend==2)
//    {
//        Util_stopClock(&dyn_id_resend_clock);
//        Util_rescheduleClock(&dyn_id_resend_clock,DYN_ID_RESEND_PERIOD);
//        dynamic_resend=0;
//    }
}

void WatchdogApp_Init(void)
{
    Watchdog_Params params;
    Watchdog_init();
    Watchdog_Params_init(&params);
    params.resetMode = Watchdog_RESET_ON;
    WatchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);

    if (WatchdogHandle == NULL) {
        /* Error opening Watchdog */
        while (1);
    }
}

static void multi_role_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
static bool firstRun = TRUE;

switch(pMsg->opcode)
{
  case L2CAP_NUM_CTRL_DATA_PKT_EVT:
  {
    /*
     * We cannot reboot the device immediately after receiving
     * the enable command, we must allow the stack enough time
     * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
     * command. This command will determine the number of
     * packets currently queued up by the LE controller.
     */
    if(firstRun)
    {
      firstRun = false;

      // We only want to set the numPendingMsgs once
      numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

      // Wait until all PDU have been sent on cxn events
      multi_role_UnRegistertToAllConnectionEvent(FOR_OAD_SEND);
      }
      break;
    }
    default:
      break;
}
}

static void multi_role_processOadWriteCB(uint8_t event, uint16_t arg)
{
  Event_post(syncEvent, event);
}
///////
