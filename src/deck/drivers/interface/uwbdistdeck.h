#ifndef __UWBDISTDECK_H__
#define __UWBDISTDECK_H__

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "libdw1000.h"
#include "stabilizer_types.h"

#include "mac.h"

// Timestamp counter frequency
#define LOCODECK_TS_FREQ (499.2e6 * 128)

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1

#define MAX_TIMEOUT portMAX_DELAY

#endif // __UWBDISTDECK_H__
