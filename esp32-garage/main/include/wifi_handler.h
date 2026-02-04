#ifndef _MIVE_WIFI_HANDLER_H
#define _MIVE_WIFI_HANDLER_H

#include "program_opaque.h"

#define WIFI_SSID "RastaviSastavi2"
#define WIFI_PWD "PUSTIMEUNUTRA"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_init_sta(mive_program_t* program);

#endif // _MIVE_WIFI_HANDLER_H