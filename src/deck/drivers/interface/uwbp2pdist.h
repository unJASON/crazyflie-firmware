#ifndef __UWB_P2P_DIST_H__
#define __UWB_P2P_DIST_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_P2P_POLL 0x01   // Poll is initiated by the tag
#define LPS_P2P_ANSWER 0x02
#define LPS_P2P_FINAL 0x03
#define LPS_P2P_REPORT 0x04 // Report contains all measurement from the anchor
#define LPS_P2P_INFORM 0X05
#define LPS_P2P_TYPE 0
#define LPS_P2P_SEQ 1

extern uwbAlgorithm_t uwbP2PDistAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsp2pTagReportPayload_t;

#define MAX_UWB_RECEIVE_TIMEOUT 65 //65ms is max interval


#endif // __UWB_P2P_DIST_H__