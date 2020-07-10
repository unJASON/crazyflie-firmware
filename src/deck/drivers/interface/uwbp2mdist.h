#ifndef __UWB_P2M_DIST_H__
#define __UWB_P2M_DIST_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_P2M_POLL 0x01   // Poll is initiated by the tag
#define LPS_P2M_ANSWER 0x02
#define LPS_P2M_FINAL 0x03
#define LPS_P2M_REPORT 0x04 // Report contains all measurement from the anchor
#define LPS_P2M_INFORM 0X05
#define LPS_P2M_UNI 0X06    
#define LPS_P2M_TYPE 0
#define LPS_P2M_SEQ 1

extern uwbAlgorithm_t uwbP2MDistAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsp2mTagReportPayload_t;

typedef struct{
  uint8_t agent_idx;  //agent's idx
  uint16_t answer_idx; //  transmission idx from agent.
  dwTime_t received_timestamp;  //receive time
} __attribute__((packed)) Agent_info;

// for each timestamp 5 Bytes is required
typedef struct {
  uint8_t group_num;
  uint16_t idx; //index of transmission
  dwTime_t last_transmission_time;  // last transmission timestamp
  Agent_info received_group[8]; //3 group member
} __attribute__((packed)) lpsp2mUNIPayload_t;

typedef struct{
  uint16_t my_transmission_idx;  // the idx of my transmission	agent 携带 用于标记 agent_last_received_time 对应的接收索引
  uint16_t agent_transmission_idx; //the idx of agent's transmission in this time. agent 本次 传输索引 
  dwTime_t my_recevied_time;    // the local timestamp when receving from agent		本机 接收 agent 本次传输 时间戳
  dwTime_t agent_last_received_time;  //data received from agent denotes the last receving timestamp.	agent 携带 上次接收本机报文 产生的 时间
  dwTime_t agent_transmit_time;  //data received from agent denotes last transmitting timestamp.	agent 携带 上次agent传输时刻的 时间戳
} __attribute__((packed)) agent_cache;



#define MAX_UWB_RECEIVE_TIMEOUT 65 //65ms is max interval
void startTransmitTimer(dwDevice_t *dev);

#endif // __UWB_P2M_DIST_H__