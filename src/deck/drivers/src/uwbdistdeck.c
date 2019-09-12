#define DEBUG_MODULE "UWBDIST"

#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "nvicconf.h"
#include "estimator.h"

#include "physicalConstants.h"

#include "uwbdistdeck.h"

#define CS_PIN DECK_GPIO_IO1

// UWB distance deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1, TX1), leaving UART1 free for use
  #define GPIO_PIN_IRQ 	GPIO_Pin_11
	#define GPIO_PIN_RESET 	GPIO_Pin_10
	#define GPIO_PORT		GPIOC
	#define EXTI_PortSource EXTI_PortSourceGPIOC
	#define EXTI_PinSource 	EXTI_PinSource11
	#define EXTI_LineN 		EXTI_Line11
	#define EXTI_IRQChannel EXTI15_10_IRQn

#define DEFAULT_RX_TIMEOUT 10000

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

static bool isInit = false;
static SemaphoreHandle_t irqSemaphore;
static SemaphoreHandle_t algoSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t rxPacket;  //last packet recevied
static packet_t txPacket;  //last packet transmitted
static volatile uint8_t curr_seq = 0;  // poll seq holder
static volatile uint8_t curr_peer = 0; // last seq of the same type (rx, tx)

static locoAddress_t uwb_peer_addrbase = 0xbccf000000000000;
static uint8_t uwb_peer_id = 0; 

float pressure, temperature, asl;
bool pressure_ok;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTwrTagReportPayload_t;

int uwb_dist_state=0;

/************ Low level ops for libdw **********/
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

  void __attribute__((used)) EXTI11_Callback(void)
	{
	  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;

	  NVIC_ClearPendingIRQ(EXTI_IRQChannel);
	  EXTI_ClearITPendingBit(EXTI_LineN);

	  //To unlock RadioTask
	  xSemaphoreGiveFromISR(irqSemaphore, &xHigherPriorityTaskWoken);

	  if(xHigherPriorityTaskWoken)
		portYIELD();
	}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
  vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};
//=============END: Low level ops for libdw===============//

static void txCallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (ANTENNA_DELAY / 2);

  switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      DEBUG_PRINT("sent LPS_TWR_POLL\n");
      poll_tx = departure;
      break;
    case LPS_TWR_FINAL:
      DEBUG_PRINT("sent LPS_TWR_FINAL\n");
      final_tx = departure;
      break;
    case LPS_TWR_ANSWER:
      DEBUG_PRINT("sent LPS_TWR_ANSWER to %02x at %04x\n", (unsigned int)txPacket.destAddress, (unsigned int)departure.low32);
      answer_tx = departure;
      break;
    case LPS_TWR_REPORT:
      DEBUG_PRINT("sent LPS_TWR_REPORT\n");
      break;
  }
}

static void rxCallback(dwDevice_t *dev)
{
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);

  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return;

  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  switch(rxPacket.payload[LPS_TWR_TYPE]) {
    case LPS_TWR_POLL:

      curr_peer = rxPacket.sourceAddress;

      int payloadLength = 2;
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      arival.full -= (ANTENNA_DELAY/2);
      poll_rx = arival;

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+payloadLength);
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      uwb_dist_state=4;
  DEBUG_PRINT("case LPS_TWR_POLL:\n");
  DEBUG_PRINT("uwb_dist_state=%d\n",uwb_dist_state);
      break;

    // Tag received messages
    case LPS_TWR_ANSWER:
      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) return;

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      arival.full -= (ANTENNA_DELAY / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      DEBUG_PRINT("dwStartTransmit LPS_TWR_FINAL\n");
      uwb_dist_state=3;
  DEBUG_PRINT("case LPS_TWR_ANSWER: \n");
  DEBUG_PRINT("uwb_dist_state=%d\n",uwb_dist_state);
      break;
    case LPS_TWR_FINAL:
      if (curr_peer != rxPacket.sourceAddress) return;

      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(txPacket.payload+2);

      arival.full -= (ANTENNA_DELAY/2);
      final_rx = arival;

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);
      report->pressure = pressure;
      report->temperature = temperature;
      report->asl = asl;
      report->pressure_ok = pressure_ok;

      dwNewTransmit(dev);
      dwSetDefaults(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwrTagReportPayload_t));
      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);
      uwb_dist_state=0;
  DEBUG_PRINT("case LPS_TWR_FINAL: \n");
  DEBUG_PRINT("uwb_dist_state=%d\n",uwb_dist_state);
      break;
    case LPS_TWR_REPORT:
    {
      DEBUG_PRINT("received LPS_TWR_REPORT\n");
      lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return;
      }

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      DEBUG_PRINT("IMPTANT: distance %0.2f\n",SPEED_OF_LIGHT * tprop);
      /*
      state.distance[current_anchor] = SPEED_OF_LIGHT * tprop;
      state.pressures[current_anchor] = report->asl;
      // Outliers rejection
      rangingStats[current_anchor].ptr = (rangingStats[current_anchor].ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats[current_anchor].history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - state.distance[current_anchor]);

      rangingStats[current_anchor].history[rangingStats[current_anchor].ptr] = state.distance[current_anchor];

      rangingOk = true;

      if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
          (diff < (OUTLIER_TH*stddev))) {
        distanceMeasurement_t dist;
        dist.distance = state.distance[current_anchor];
        dist.x = options->anchorPosition[current_anchor].x;
        dist.y = options->anchorPosition[current_anchor].y;
        dist.z = options->anchorPosition[current_anchor].z;
        dist.stdDev = 0.25;
        estimatorEnqueueDistance(&dist);
      }

      ranging_complete = true;
      */
      uwb_dist_state=0;
  DEBUG_PRINT("case LPS_TWR_REPORT:\n"); 
  DEBUG_PRINT("uwb_dist_state=%d\n",uwb_dist_state);
      break;
    }
  }
//  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev)
{
  DEBUG_PRINT("initiateRanging\n");
  dwIdle(dev);

  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;
  // no need to initialize RX buffer

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  curr_peer = 0;

  pressure = temperature = asl = 0;
  pressure_ok = true;

  uwb_peer_id = 0;

  txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
  txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;

  txPacket.sourceAddress = uwb_peer_addrbase|uwb_peer_id;
  txPacket.destAddress = uwb_peer_addrbase|uwb_peer_id;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static void uwbTask(void* parameters)
{
  dwDevice_t* dev=dwm;
  int time2wait=0;
  dwSetReceiveWaitTimeout(dev, 3000);//~1ms
  dwCommitConfiguration(dev);

  systemWaitStart();
  vTaskDelay(3000/portTICK_PERIOD_MS);
  //unsigned int i=0;
  while(1) {
        initiateRanging(dev);
    vTaskDelay(3000/portTICK_PERIOD_MS);
  }
        setRadioInReceiveMode(dev);
  while(1) {
  #if 0
    if(i>2000) {i=0; uwb_dist_state=1;} else i++;
    switch(uwb_dist_state) {
      case 0:
      case 2:
      case 3:
      case 4:
        setRadioInReceiveMode(dev);
        time2wait=1;
        break;
      case 1:
        initiateRanging(dev);
        time2wait=10/portTICK_PERIOD_MS;
        uwb_dist_state = 2;
        break;
      default:
        time2wait=1/portTICK_PERIOD_MS;
    }
  #endif
    if (xSemaphoreTake(irqSemaphore, time2wait)) {
      do{
        xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwHandleInterrupt(dwm);
        xSemaphoreGive(algoSemaphore);
      } while(digitalRead(GPIO_PIN_IRQ) != 0);
    }
  }
}
 
static void uwbdistInit(DeckInfo *info)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  spiBegin();

  // Init IRQ input
  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IRQ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init reset output
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RESET;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  // Init CS pin
  pinMode(CS_PIN, OUTPUT);

  // Reset the DW1000 chip
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));

  // Initialize the driver
  dwInit(dwm, &dwOps);       // Init libdw

  int result = dwConfigure(dwm);
  if (result != 0) {
    isInit = false;
    DEBUG_PRINT("Failed to configure DW1000!\n");
    return;
  }

  dwEnableAllLeds(dwm);

  dwTime_t delay = {.full = 0};
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txCallback);
  dwAttachReceivedHandler(dwm, rxCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);


  #ifdef LPS_LONGER_RANGE
  dwEnableMode(dwm, MODE_SHORTDATA_MID_ACCURACY);
  #else
  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
  #endif

  dwSetChannel(dwm, CHANNEL_2);
  dwUseSmartPower(dwm, true);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwSetReceiveWaitTimeout(dwm, DEFAULT_RX_TIMEOUT);
  dwCommitConfiguration(dwm);

  // Enable interrupt
  NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_VERY_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  vSemaphoreCreateBinary(irqSemaphore);
  vSemaphoreCreateBinary(algoSemaphore);

  xTaskCreate(uwbTask, "lps", 3*configMINIMAL_STACK_SIZE, NULL,
                    5/*priority*/, NULL);

  isInit = true;
}

static bool uwbdistTest()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM1000\n");
  }

  return isInit;
}

static const DeckDriver uwbdist_deck = {
  .vid = 0xBC,
  .pid = 0x06, //0x06,
  .name = "bcUWBdist",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,
  .requiredLowInterferenceRadioMode = true,

  .init = uwbdistInit,
  .test = uwbdistTest,
};

DECK_DRIVER(uwbdist_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcUWBdist, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(uwbdist)
PARAM_ADD(PARAM_UINT8, uwbPeerID, &uwb_peer_id)
PARAM_GROUP_STOP(uwbdist)
