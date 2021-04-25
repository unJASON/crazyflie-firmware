#include <string.h>
#include "uwbp2mdist.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "crtp_localization_service.h"
#include "physicalConstants.h"
#include "configblock.h"
#include "semphr.h"
#include "timers.h"
#include <stdlib.h>
#include "locdeck_antenna_delay.h"
#include "pos_localization.h"
#include "deck.h"
#include "system.h"
#include "commander.h"

#define ANTENNA_OFFSET 154.33   // In meter

#define NUM_UAV 10       //当前无人机数量
#define NUM_CYC 1000      // number of transmit cycle stored in memory
static int ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;


static int agent_countdown[NUM_UAV]={0,0,0,0,0,0,0,0,0,0}; //countdown for transmitting
static float distances[NUM_UAV]={10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0};
static SemaphoreHandle_t positionSemaphore;
static float positions[3][NUM_UAV]={
                                    {0,0,0,0,0,0,0,0,0,0},
                                    {0,0,0,0,0,0,0,0,0,0},
                                    {0,0,0,0,0,0,0,0,0,0}
                                    };
static float positions_cov[2][NUM_UAV]={
                                    {10,10,10,10,10,10,10,10,10,10},
                                    {10,10,10,10,10,10,10,10,10,10}
                                    };
static bool refresh[NUM_UAV]={false,false,false,false,false,false,false,false,false,false};

static locoAddress_t fullAddress[NUM_UAV]={
  0xbccf000000000000|24,0xbccf000000000000|28,0xbccf000000000000|33,
  0xbccf000000000000|37,0xbccf000000000000|1,0xbccf000000000000|2,
  0xbccf000000000000|5,0xbccf000000000000|4,0xbccf000000000000|3,
  0xbccf000000000000|6}; 
static bool isVisit[NUM_UAV] = {false,false,false,false,false,false,false,false,false,false};

static locoAddress_t myAddress;  //store my ownAddress27
static uint8_t my_addr_idx;             //store my index

static packet_t txPacket;   //发送的包



static volatile uint16_t curr_seq = 0;  //序号

static int64_t MAX_TIMESTAMP = 1099511627776; //2**40
static uint32_t default_twr_interval=80000;   
static uint32_t timeout_p2m=80000;

static dwTime_t transmit_timer[NUM_CYC]; //stores recent NUM_CYC transmission timestamp

static agent_cache old_cache[NUM_UAV];  // stores the old information
static agent_cache new_cache[NUM_UAV];  // stores the received information

static SemaphoreHandle_t visitSemaphore;
static Agent_info  received_agent[NUM_UAV];  //stores the information received from others

void getpositionInfo(position_state *ps,bool *refresh_flag,int ps_num){
  while (true)
  {
    if( xSemaphoreTake(positionSemaphore, portMAX_DELAY)) {
        break;
    }
  }
  for (uint8_t i = 0;i< ps_num ;i++){
    if (i != my_addr_idx){
      //get neighbor position
      ps[i].x = positions[0][i];
      ps[i].y = positions[1][i];
      ps[i].z = positions[2][i];
      ps[i].P[0][0] = positions_cov[0][i];
      ps[i].P[1][1] = positions_cov[1][i];
      ps[i].distance=distances[i];
      refresh_flag[i] = refresh[i];
      //reset flag
      refresh[i] = false;
      // DEBUG_PRINT("%d : %f %f %f\n",i,(double)positions[0][i],(double)positions[1][i],(double)positions[2][i]);
    }else{
      // set my position for broadcast
      positions[0][i] = ps[i].x;
      positions[1][i] = ps[i].y;
      positions[2][i] = ps[i].z;
      positions_cov[0][i] = ps[i].P[0][0];
      positions_cov[1][i] = ps[i].P[1][1];
    }
  }
  xSemaphoreGive(positionSemaphore);
};
uint8_t getmy_idx(){
  return my_addr_idx;
}

//建立一个映射关系，知道地址，去逻辑下标
static int findIndex(locoAddress_t find)
{

  for(int i=0;i<NUM_UAV;i++)
    if(fullAddress[i]==find) 
      return i;
  // collision happened
  return -1;
}
static dwTime_t departure;
static void txcallback(dwDevice_t *dev)    //发送报文的回调函数
{
  dwGetTransmitTimestamp(dev, &departure);   //获取发送时间戳
  departure.full += (ANTENNA_DELAY / 2);      //加入天线延迟，否则计算结果差距很大
  transmit_timer[curr_seq] = departure;     //store the leaving timestamp 

  curr_seq = (curr_seq + 1) % NUM_CYC;
}

static int64_t temp,temp_end,temp_start;
static int64_t difference1,difference2;      //difference
static Agent_info agent;
static int64_t tround1,treply1,treply2,tround2;
static float tprop;
static int64_t tprop_ctn;
static int check_index;
static int i;
static packet_t rxPacket;
static int cache_index;
static uint32_t rxcallback(dwDevice_t *dev)   //收到报文的回调函数
{  
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);    //获取收到报文时的时间戳


  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) {
    // in case the error packet
    dwNewReceive(dev);
    // dwSetDefaults(dev);
    dwStartReceive(dev);
    return timeout_p2m;
  }

  // accessing critical resource
   while (true)
  {
    if( xSemaphoreTake(positionSemaphore, portMAX_DELAY)) {
        break;
    }
  }
  while (true){
    if( xSemaphoreTake(visitSemaphore, portMAX_DELAY)) {
        break;
    }
  }
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  arival.full -= (ANTENNA_DELAY/2);
  lpsp2mUNIPayload_t *report = (lpsp2mUNIPayload_t *)(rxPacket.payload);

  cache_index = rxPacket.sourceAddress; // index from received packet
  if (cache_index == my_addr_idx ||cache_index >= NUM_UAV || cache_index <0)
{
    dwNewReceive(dev);
    // dwSetDefaults(dev);
    dwStartReceive(dev);
    xSemaphoreGive(visitSemaphore);
    xSemaphoreGive(positionSemaphore);
    return timeout_p2m;
  }
  isVisit[cache_index] = true;    //set the visit flag, semaphore is required in the future.
  received_agent[cache_index].agent_idx = cache_index;
  received_agent[cache_index].received_timestamp = arival;
  received_agent[cache_index].answer_idx = report->idx;

  //***************
  bool isContain = false;
  for (i = 0; i < report->group_num;i++){
    agent = report->received_group[i];
    if ( agent.agent_idx == my_addr_idx ){
      //check and do some substitution
      isContain = true;
      check_index = (old_cache[cache_index].agent_transmission_idx +1)%NUM_CYC;
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
          
          // modify the formulation to deal with the overflow problem.
          temp_end = answer_rx.high8;
          temp_end = temp_end<<32;
          temp_start = poll_tx.high8;
          temp_start= temp_start<<32;
          temp = (MAX_TIMESTAMP+temp_end + answer_rx.low32 - temp_start - poll_tx.low32)%MAX_TIMESTAMP;
          tround1 = temp;
          

          temp_end = answer_tx.high8;
          temp_end = temp_end<<32;
          temp_start=poll_rx.high8;
          temp_start = temp_start<<32;
          temp = (MAX_TIMESTAMP+temp_end + answer_tx.low32 - temp_start - poll_rx.low32)%MAX_TIMESTAMP;
          treply1 = temp;
          
          temp_end=final_rx.high8;
          temp_end = temp_end<<32;
          temp_start= answer_tx.high8;
          temp_start = temp_start<<32;
          temp = (MAX_TIMESTAMP+temp_end + final_rx.low32 -temp_start - answer_tx.low32)%MAX_TIMESTAMP;
          tround2 = temp;
         
          temp_end=final_tx.high8;
          temp_end = temp_end<<32;
          temp_start=answer_rx.high8;
          temp_start = temp_start<<32;
          temp = (MAX_TIMESTAMP+temp_end + final_tx.low32 - temp_start - answer_rx.low32)%MAX_TIMESTAMP;
          treply2 = temp;

          
          
          //overflow: tround1 * tround2 and  treply1*treply2 
          difference1 = tround1 - treply1;
          difference2 = tround2 - treply2;
          tprop_ctn = (difference1 * treply2 + difference2*treply1 + difference2* difference1)/( (tround1 + tround2 + treply1 + treply2));
          
          
          if (tprop_ctn >= 0 && tprop_ctn <=3200) {
              // tprop = tprop_ctn/LOCODECK_TS_FREQ;
              // distances[cache_index] = SPEED_OF_LIGHT * tprop;
              tprop = tprop_ctn *(0.004691);
              distances[cache_index] = distances[cache_index]*0.6f+tprop*0.4f;
              positions[0][cache_index]=report->x;
              positions[1][cache_index]=report->y;
              positions[2][cache_index]=report->z;
              positions_cov[0][cache_index]=report->cov_xx;
              positions_cov[1][cache_index]=report->cov_yy;
              refresh[cache_index]=true;
          }else{
              // impossible situation
              // distances[cache_index] = -0.1;
          }

      }else{
        // case 4:
        //  do nothing, just replacement
      }
      // do replacement
      old_cache[cache_index].my_recevied_time = new_cache[cache_index].my_recevied_time;
      old_cache[cache_index].my_transmission_idx = new_cache[cache_index].my_transmission_idx;
      old_cache[cache_index].agent_transmission_idx=new_cache[cache_index].agent_transmission_idx;
      old_cache[cache_index].agent_last_received_time=new_cache[cache_index].agent_last_received_time;
    }
  }
  // case 1,2 replace the old_cache
  if (!isContain){
      old_cache[cache_index].my_recevied_time = arival;
      old_cache[cache_index].agent_transmission_idx = report->idx;
  }
 
  
  
  dwNewReceive(dev);
  // dwSetDefaults(dev);
  dwStartReceive(dev);
  // release semaphore
  xSemaphoreGive(visitSemaphore);
  xSemaphoreGive(positionSemaphore);
  return timeout_p2m;
}




// ***********runTransmit()****************
static TimerHandle_t Transmit_timer_handle = NULL;
static dwDevice_t *dev_timer;
static int chosen_group[GROUP_SIZE]={-1,-1,-1,-1,-1,-1,-1,-1};
static int const_p[NUM_UAV] = {20,20,20,20,15,15,15,15,15,15};
static int rand_p[NUM_UAV]={11,11,11,11,81,81,81,81,81,81};
uint8_t swap = 0,tmp = 0;
// ************runTransmit()***************
static void runTransmit(){

  txPacket.destAddress=my_addr_idx;
  txPacket.sourceAddress=my_addr_idx;
  lpsp2mUNIPayload_t *report =(lpsp2mUNIPayload_t *)(txPacket.payload);
  report->idx = curr_seq;
  report->last_transmission_time = transmit_timer[ (curr_seq+NUM_CYC-1)%NUM_CYC ];
  uint8_t len = 0;
  // accessing critical resource
  while (true){
    if( xSemaphoreTake(positionSemaphore, portMAX_DELAY)) {
        break;
    }
  }

  while (true){
    if( xSemaphoreTake(visitSemaphore, portMAX_DELAY)) {
        break;
    }
  }
  

  //choose minimun top K
  uint8_t taken = 0;
  for(uint8_t i = 0; i < NUM_UAV; i++){
    if( (i != my_addr_idx) && isVisit[i]){
      isVisit[i] = false;
      // only calculate visited one's period
      if(taken <GROUP_SIZE){
        chosen_group[taken] = i;
        taken +=1;
      }else{
        // if full then swap the max
        swap = i;
        for (uint8_t j = 0; j < GROUP_SIZE; j++){
          if(agent_countdown[swap] < agent_countdown[chosen_group[j]]){
            tmp = chosen_group[j];
            chosen_group[j]=swap;
            swap = tmp;
          }
        }
      }
    }
  }
  //reset isVisit waiting period
  for (uint8_t i = 0;i<taken;i++){
    memcpy(&report->received_group[i],&received_agent[chosen_group[i]],sizeof(Agent_info));
    agent_countdown[chosen_group[i]]= const_p[chosen_group[i]]+ rand_p[chosen_group[i]]/2;
  }
  len = taken;
  int period = const_p[my_addr_idx] + rand()%rand_p[my_addr_idx];
  for (uint8_t i = 0; i < NUM_UAV; i++)
  {
    if(agent_countdown[i]>=-10000){
      agent_countdown[i] = agent_countdown[i] - period;
    }
  }
  report->group_num = len;
  //add position
  report->x = positions[0][my_addr_idx];
  report->y = positions[1][my_addr_idx];
  report->z = positions[2][my_addr_idx];
  report->cov_xx=positions_cov[0][my_addr_idx];
  report->cov_yy=positions_cov[1][my_addr_idx];

  xTimerChangePeriod(Transmit_timer_handle,M2T(period),10000);
  dwNewTransmit(dev_timer);
  // dwSetDefaults(dev_timer);
  dwSetData(dev_timer, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+sizeof(lpsp2mUNIPayload_t));
  dwWaitForResponse(dev_timer, true);
  dwStartTransmit(dev_timer);
  xSemaphoreGive(visitSemaphore);
  xSemaphoreGive(positionSemaphore);
  // *****************************
}


void startTransmitTimer(dwDevice_t *dev){
  // create a binary semaphore for lock.
  vSemaphoreCreateBinary(visitSemaphore);
  vSemaphoreCreateBinary(positionSemaphore);
  if (visitSemaphore == NULL){
    FAIL_PRINT("semaphore create fail\n");
  }
  dev_timer = dev;
  int channel = configblockGetRadioChannel();
   //获取自身物理信息关联的ip
  myAddress= 0xbccf000000000000|channel;  //这个值每次都要动态的改
  my_addr_idx = (uint8_t)findIndex(myAddress);
  ANTENNA_DELAY = ( (UAV_LOC_DELAY[channel])*499.2e6*128 )/299792458.0;
  vTaskDelay(5000);
  

  Transmit_timer_handle = xTimerCreate("P2M_transmit_timer", M2T(150), pdFALSE, (void*)20, runTransmit);// run the transmit function
  if(Transmit_timer_handle != NULL){
    xTimerStart(Transmit_timer_handle, 0);
  }
}




static void initiateRanging(dwDevice_t *dev)
{
  dwNewReceive(dev);
  // dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t p2mDistOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventReceiveTimeout:
      dwNewReceive(dev);
      // dwSetDefaults(dev);
      dwStartReceive(dev);
      break;
    case eventTimeout:  // 一直收不到，则重启全部对话
      initiateRanging(dev);
      break;
    case eventReceiveFailed:
      initiateRanging(dev);
      break;
  }
  return default_twr_interval;
}




static void p2mDistInit(dwDevice_t *dev)
{
  return;
  // Initialize the packet in the TX buffer
  // memset(&txPacket, 0, sizeof(txPacket));
  // MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  // txPacket.pan = 0xbccf;

  // dwSetReceiveWaitTimeout(dev, 0);
  // dwCommitConfiguration(dev);
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
LOG_ADD(LOG_FLOAT, distance2peer40,&distances[3])
LOG_ADD(LOG_FLOAT, distance2peer50,&distances[4])
LOG_GROUP_STOP(peerdist)

LOG_GROUP_START(peerdist2)
LOG_ADD(LOG_FLOAT, distance2peer60,&distances[5])
LOG_ADD(LOG_FLOAT, distance2peer70,&distances[6])
LOG_ADD(LOG_FLOAT, distance2peer80,&distances[7])
LOG_ADD(LOG_FLOAT, distance2peer90,&distances[8])
LOG_ADD(LOG_FLOAT, distance2peer90,&distances[9])
LOG_GROUP_STOP(peerdist2)