#include <string.h>
#include "uwbp2mdist.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "crtp_localization_service.h"
#include "physicalConstants.h"
#include "configblock.h"
#include "console.h"
#include "timers.h"

#define ANTENNA_OFFSET 154.6   // In meter

#define NUM_UAV 3       //当前无人机数量
#define NUM_CYC 100      // number of transmit cycle stored in memory
static int ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static float distances[NUM_UAV]={0.0,0.0,0.0};

static locoAddress_t fullAddress[NUM_UAV]={0xbccf000000000000|50,0xbccf000000000000|85,0xbccf000000000000|95}; 
static locoAddress_t myAddress;  //store my ownAddress
static packet_t txPacket;   //发送的包

static volatile uint8_t curr_seq = 0;  //序号


static uint32_t timeout_p2m=0;
static uint32_t default_twr_interval=8000;   //设置最大超时重传poll报文的时间

static dwTime_t transmit_timer[NUM_CYC]; //stores recent NUM_CYC transmission timestamp

static agent_cache old_cache[NUM_UAV];  // stores the old information
static agent_cache new_cache[NUM_UAV];  // stores the received information
static bool isVisit[NUM_UAV] = {false,false,false};
static Agent_info  received_agent[NUM_UAV];  //stores the information received from others
//建立一个映射关系，知道地址，去逻辑下标
static int findIndex(locoAddress_t find)
{
  for(int i=0;i<NUM_UAV;i++)
    if(fullAddress[i]==find) 
      return i;
  return -1;
}

static void txcallback(dwDevice_t *dev)    //发送报文的回调函数
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);   //获取发送时间戳
  departure.full += (ANTENNA_DELAY / 2);      //加入天线延迟，否则计算结果差距很大
  transmit_timer[curr_seq] = departure;     //store the leaving timestamp 
// DEBUG_PRINT("m:%lu\n",time_bias.low32);
// DEBUG_PRINT("d:%lu %d\n",departure.high32,departure.low8);
  curr_seq = (curr_seq + 1) % NUM_CYC;      
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}


static uint32_t rxcallback(dwDevice_t *dev)   //收到报文的回调函数
{ 
  
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);    //获取收到报文时的时间戳
  
  uint8_t temp;
  uint32_t bias32;
  uint8_t bias8;
  memset(&bias8,-1,sizeof(uint8_t));
  memset(&bias32,-1,sizeof(uint32_t));

  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) {
    // in case the error packet
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return 0;
  }

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  
  int my_index = findIndex(myAddress);
  int cache_index = findIndex(rxPacket.sourceAddress);
  if (my_index == cache_index){
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return timeout_p2m;
  }
  arival.full -= (ANTENNA_DELAY/2);
  lpsp2mUNIPayload_t *report = (lpsp2mUNIPayload_t *)(rxPacket.payload);
  
  isVisit[cache_index] = true;    //set the visit flag, semaphore is required in the future.
  received_agent[cache_index].agent_idx = cache_index;
  received_agent[cache_index].received_timestamp = arival;
  received_agent[cache_index].answer_idx = report->idx;

  for (int i = 0; i < report->group_num;i++){
    Agent_info agent = report->received_group[i];
    if ( agent.agent_idx == (uint8_t)my_index ){
      //check and do some substitution
      
      int check_index = (old_cache[cache_index].agent_transmission_idx +1)%NUM_CYC;
      
      new_cache[cache_index].my_recevied_time = arival;
      new_cache[cache_index].my_transmission_idx = agent.answer_idx;
      new_cache[cache_index].agent_transmission_idx = report->idx;
      new_cache[cache_index].agent_last_received_time = agent.received_timestamp;
      if (report->idx == check_index){
        // do caculation
          
          memcpy(&old_cache[cache_index].agent_transmit_time,&report->last_transmission_time,5);
          poll_tx = transmit_timer[old_cache[cache_index].my_transmission_idx];
          poll_rx = old_cache[cache_index].agent_last_received_time;
          answer_tx = old_cache[cache_index].agent_transmit_time;
          answer_rx = old_cache[cache_index].my_recevied_time;
          final_tx = transmit_timer[new_cache[cache_index].my_transmission_idx];
          final_rx = new_cache[cache_index].agent_last_received_time;
          double tround1,treply1,treply2,tround2,tprop_ctn,tprop;

          // modify the formulation to deal with the overflow problem.
          temp = (answer_rx.high8 - poll_tx.high8 + bias8+1) % (bias8+1);
          tround1 = temp*(bias32+1)+ (answer_rx.low32 - poll_tx.low32);
          temp = (answer_tx.high8 - poll_rx.high8+ bias8+1 ) % (bias8+1);
          treply1 = temp*(bias32+1)+ (answer_tx.low32 - poll_rx.low32);
          temp = (final_rx.high8 - answer_tx.high8+ bias8+1) % (bias8+1);
          tround2 = temp*(bias32+1)+(final_rx.low32 - answer_tx.low32);
          temp = (final_tx.high8 - answer_rx.high8+ bias8+1)% (bias8+1) ;
          treply2 = temp*(bias32+1)+(final_tx.low32 - answer_rx.low32);
          tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);          
          tprop = tprop_ctn/LOCODECK_TS_FREQ;
          distances[cache_index] = SPEED_OF_LIGHT * tprop;
      }else{
        // do nothing
      }
      // do replacement
      old_cache[cache_index].my_recevied_time = new_cache[cache_index].my_recevied_time;
      old_cache[cache_index].my_transmission_idx = new_cache[cache_index].my_transmission_idx;
      old_cache[cache_index].agent_transmission_idx=new_cache[cache_index].agent_transmission_idx;
      old_cache[cache_index].agent_last_received_time=new_cache[cache_index].agent_last_received_time;
    }
  }
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
  return timeout_p2m;
}


static void initiateRanging(dwDevice_t *dev)     //在这个函数中实现指定第一架飞机发起poll报文，开启整个会话
{
  dwIdle(dev);
  //获取自身物理信息关联的ip
  int myChanel = configblockGetRadioChannel();
  myAddress=0xbccf000000000000|myChanel;  //这个值每次都要动态的改
  
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t p2mDistOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      timeout_p2m=default_twr_interval;   //收到包，重置时钟
      rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventReceiveTimeout:   
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);
      timeout_p2m = (timeout_p2m>MAX_UWB_RECEIVE_TIMEOUT ? timeout_p2m-MAX_UWB_RECEIVE_TIMEOUT : 0) ;
      break;
    case eventTimeout:  // 一直收不到，则重启全部对话
      initiateRanging(dev);
      timeout_p2m=default_twr_interval;
      break;
    case eventReceiveFailed:
      return 0;
  }
  return timeout_p2m;
}


static TimerHandle_t Transmit_timer_handle = NULL;
static dwDevice_t *dev_timer;
static void runTransmit(){
  txPacket.destAddress=myAddress;
  txPacket.sourceAddress=myAddress;
  lpsp2mUNIPayload_t *report =(lpsp2mUNIPayload_t *)(txPacket.payload);
  report->idx = curr_seq;
  report->last_transmission_time = transmit_timer[ (curr_seq+NUM_CYC-1)%NUM_CYC ];
  uint8_t len = 0;
  for (int i= 0 ;i<NUM_UAV;i++){
    if(isVisit[i]){
        memcpy(&report->received_group[len],&received_agent[i],sizeof(Agent_info));
        len = len+1;
        isVisit[i] = false;
    }
  }
  report->group_num = len;
  dwNewTransmit(dev_timer);
  dwSetDefaults(dev_timer);
  dwSetData(dev_timer, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+sizeof(lpsp2mUNIPayload_t));
  // dwWaitForResponse(dev_timer, true);
  dwStartTransmit(dev_timer);
}


void startTransmitTimer(dwDevice_t *dev){
  dev_timer = dev;
  Transmit_timer_handle = xTimerCreate("P2M_transmit_timer", M2T(100), pdTRUE, (void*)20, runTransmit);
  if(Transmit_timer_handle != NULL){
    xTimerStart(Transmit_timer_handle,0);
  }
}

static void p2mDistInit(dwDevice_t *dev)
{
  return;
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  dwSetReceiveWaitTimeout(dev, 0);
  dwCommitConfiguration(dev);
}

static bool isRangingOk()
{
  return true;
}

point_t anchorPosition[2];
static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
    *position = anchorPosition[anchorId];
    return true;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return 2;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return 2;
}
uwbAlgorithm_t uwbP2MDistAlgorithm = {
  .init = p2mDistInit,
  .onEvent = p2mDistOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};


//用于向client传送日志数据
LOG_GROUP_START(peerdist)
LOG_ADD(LOG_FLOAT, distance2peer10,&distances[0])
LOG_ADD(LOG_FLOAT, distance2peer20,&distances[1])
LOG_ADD(LOG_FLOAT, distance2peer30,&distances[2])
LOG_GROUP_STOP(peerdist)