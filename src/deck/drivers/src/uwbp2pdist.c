
#include <string.h>
#include "uwbp2pdist.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "crtp_localization_service.h"
#include "physicalConstants.h"
#define ANTENNA_OFFSET 154.6   // In meter
#define NUM_UAV 3       //当前无人机数量


static int ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
static float distances[NUM_UAV]={0.0,0.0,0.0}
static locoAddress_t fullAddress[NUM_UAV]={0xbccf000000000000|10，0xbccf000000000000|20，0xbccf000000000000|30}; 
static locoAddress_t myAddress;  //store my ownAddress
static packet_t txPacket;   //发送的包

static volatile uint8_t curr_seq = 0;  //序号用于标记一次测距，防止错乱

float pressure = 0;        
float temperature = 0;
float asl = 0;
bool pressure_ok = true;

static uint32_t timeout_p2p=0;
static uint32_t default_twr_interval=8000;   //设置最大超时重传poll报文的时间
locoAddress_t nextAddress(locoAddress_t s)    //用于返回下一架无人机中的地址，我们将逻辑顺序暂时写死
{
   for(int i=0;i<NUM_UAV;i++)
   {
      if(fullAddress[i]==s)
        return fullAddress[(i+1)%NUM_UAV];
   }
   return NULL;
}

//建立一个映射关系，知道地址，去逻辑下标
int findIndex(locoAddress_t find)
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

  switch (txPacket.payload[0]) {
    case LPS_P2P_POLL:
      //DEBUG_PRINT("sent LPS_P2P_POLL\n");
      poll_tx = departure;
      break;
    case LPS_P2P_FINAL:
      //DEBUG_PRINT("sent LPS_P2P_FINAL\n");
      final_tx = departure;
      break;
    case LPS_P2P_ANSWER:
      //DEBUG_PRINT("sent LPS_P2P_ANSWER to %02x at %04x\n", (unsigned int)txPacket.destAddress, (unsigned int)departure.low32);
      answer_tx = departure;
      break;
    case LPS_P2P_REPORT:
      //DEBUG_PRINT("sent LPS_P2P_REPORT\n");
      break;
    case LPS_P2P_INFORM:
      //DEBUG_PRINT("sent LPS_P2P_INFORM\n")
      break;
  }
}


static uint32_t rxcallback(dwDevice_t *dev)   //收到报文的回调函数
{
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);    //获取收到报文时的时间戳

  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return 0;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  DEBUG_PRINT("F %d\t", (unsigned int)txPacket.destAddress);

  switch(rxPacket.payload[LPS_P2P_TYPE]) {  //负载中第一字节是表示报文类型
    case LPS_P2P_POLL:     //最先检查地址，不是自己的则不处理
      if(rxPacket.destAddress!=myAddress) return 0;
DEBUG_PRINT("POLL\n");
      txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_ANSWER;
      txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];
      txPacket.destAddress=rxPacket.sourceAddress;
      txPacket.sourceAddress=myAddress;
      arival.full -= (ANTENNA_DELAY/2);   //减去天线传输时延
      poll_rx = arival;
      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;

    case LPS_P2P_ANSWER:   //answer 报文是必须要要做地址解析的,但是final报文是不能携带计算数据的
    if(rxPacket.destAddress!=myAdress||rxPacket.payload[LPS_P2P_SEQ]!=curr_seq) return 0;
DEBUG_PRINT("ANS\n");

      txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_FINAL;
      txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];
      txPacket.destAddress = rxPacket.sourceAddress;
      txPacket.sourceAddress = rxPacket.destAddress;
     
      arival.full -= (ANTENNA_DELAY / 2);
      answer_rx = arival;
      
      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2));  //将做需要的计算数据传输过去
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      break;
    case LPS_P2P_FINAL:   //现在改成收到finnal报文报文时计算数据,注意final报文时测距方发出，被测距的收到，所以不检测seq
DEBUG_PRINT("FIN\n");
      if(rxPacket.destAddress!=myAdress) return 0;
      
     lpsp2pTagReportPayload_t *report = (lpsp2pTagReportPayload_t *)(txPacket.payload+2);

      arival.full -= (ANTENNA_DELAY/2);
      final_rx = arival;

      txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_REPORT;
      txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];
      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);
      report->pressure = pressure;
      report->temperature = temperature;
      report->asl = asl;
      report->pressure_ok = pressure_ok;

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsp2pTagReportPayload_t));
      //dwWaitForResponse(dev, true);   现在只需要report就行，不需要等回应
      dwStartTransmit(dev);
      //timeout_p2p = default_twr_interval/2; //set a shorter delay to sent next poll
      break;
    case LPS_P2P_REPORT:     /*目的地址不是自己。令牌环不是自己的话，就不管，收到report论文时，如果目的地址是我，则接受距离信息，并存储为相对应的距离
                              如果令牌环是自己，但目的地址不是自己的话，则说明上一家已经完成数据采集了*/
    {                         
DEBUG_PRINT("REPT\n"); 
      if(rxPacket.destAddress!=myAddres||rxPacket.payload[LPS_P2P_SEQ]!=curr_seq)  return 0;   

      //是别人回传距离数据的
      //保存对应的距离
      
      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2)
      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      distances[findIndex(rxPacket.sourceAddress)] = SPEED_OF_LIGHT * tprop;   //将距离存在适当的位置
    
DEBUG_PRINT("d=%d\n",(int)(100*pdistance));
      if(nextAddress(rxPacket.sourceAddress)!=myAddress)  //距离信息还没收集满,因为最后一次采集是我的逻辑上家,则发送一次poll报文
      {
        txPacket.sourceAddress = myAddress; 
        txPacket.destAddress = nextAddress(rxPacket.sourceAddress);
        txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_POLL;
        txPacket.payload[LPS_P2P_SEQ] = ++curr_seq;  //这里是非常重要的，保证是同一次测距
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      }
      else    //距离信息已经收集满，则发inform报文，通知下一家开始采集邻居数据
      {
        txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_INFORM;
        txPacket.payload[LPS_P2P_SEQ] = rxPacket.payload[LPS_P2P_SEQ];
        txPacket.sourceAddress = myAddress;
        txPacket.destAddress = nextAddress(address,NUM_UAV,myaddress);  //逻辑环路中的下一架无人机
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2)；  //
        //dwWaitForResponse(dev, true);   这里先暂时不考虑接受回应，只发通知就好了
        dwStartTransmit(dev);
      }
      }
      break;
      case LPS_P2P_INFORM:               //假如是通知报文的话，如果是我的地址，我将修改令牌(逻辑下家)，开始向逻辑下家发送poll报文
      {
        if(rxPacket.desAddress!=myaddress)  //如果不是给我的通知，则丢弃
          return 0；
        else     //是给我的通知
        {
          txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_POLL;
          txPacket.payload[LPS_P2P_SEQ] = ++curr_seq;
          txPacket.destAddress=nextAddress(myAddress);  //允许逻辑下家和我通信
          txPacket.sourceAddress = myaddress;
          dwNewTransmit(dev);
          dwSetDefaults(dev);
          dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2)；  //+4 for pass back distance
          dwWaitForResponse(dev, true);
          dwStartTransmit(dev);
        }
      }
      break;
    }
  }
  return timeout_p2p;
}

static void initiateRanging(dwDevice_t *dev)     //在这个函数中实现指定第一架飞机发起poll报文，开启整个会话
{
  dwIdle(dev);

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;

  pressure = temperature = asl = 0;
  pressure_ok = true;

  //获取自身物理信息关联的ip
  myAddress=0xbccf000000000000|10;  //这个值每次都要动态的改
  if(myAddress!=address[0])   //如果不是逻辑第一架，则不发送poll报文
    return ;
  txPacket.sourceAddress = address[0]; //poll报文是广播，可不写目的地址
  txPacket.destAddress = 0xbccf000000000000;
  txPacket.payload[LPS_P2P_TYPE] = LPS_P2P_POLL;
  txPacket.payload[LPS_P2P_SEQ] = ++curr_seq;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);

  DEBUG_PRINT("T %x\n", (unsigned int)txPacket.destAddress);
}

static uint32_t p2pDistOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
      //DEBUG_PRINT("ePR\n"),
      timeout_p2p=default_twr_interval;   //收到包，重置时钟
      rxcallback(dev);
      break;
    case eventPacketSent:
      DEBUG_PRINT("ePS\n"),
      txcallback(dev);
      break;
    case eventReceiveTimeout:   
      DEBUG_PRINT("eRT\n"),
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);
      timeout_p2p = (timeout_p2p>MAX_UWB_RECEIVE_TIMEOUT ? timeout_p2p-MAX_UWB_RECEIVE_TIMEOUT : 0) ;
      break;
    case eventTimeout:  // 一直收不到，则重启全部对话
      DEBUG_PRINT("eT\n"),
      initiateRanging(dev);
      timeout_p2p=default_twr_interval;
      break;
    case eventReceiveFailed:
      return 0;
  }
  return timeout_p2p;
}

static void p2pDistInit(dwDevice_t *dev)
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
uwbAlgorithm_t uwbP2PDistAlgorithm = {
  .init = p2pDistInit,
  .onEvent = p2pDistOnEvent,
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