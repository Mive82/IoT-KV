#ifndef _MIVE_GARAGE_H
#define _MIVE_GARAGE_H

#include "esp_err.h"
#include "driver/i2c_master.h"

union garage_i2c_reg
{
	uint8_t reg;
	struct {
		uint8_t state : 7;
		uint8_t command : 1;
	} s;
};

typedef struct mive_garage_t 
{
  i2c_master_bus_handle_t bus_master;
  i2c_master_dev_handle_t dev_handle;
  union garage_i2c_reg reg;
} mive_garage_t;



enum garage_state_e
{
	// Invalid state, not supposed to appear
	GARAGE_INVALID = 0,
	// Stopped states -> Next state is opening
	GARAGE_CLOSED = 1,
	GARAGE_CLOSING_STOPPED,
	// Stopped states -> Next state is closing
	GARAGE_OPEN,
	GARAGE_OPENING_STOPPED,

	// Moving states
	GARAGE_OPENING,
	GARAGE_CLOSING,

  GARAGE_STATE_MAX = GARAGE_CLOSING,
};

esp_err_t mive_garage_init(mive_garage_t* garage, i2c_master_bus_config_t* bus_config, i2c_device_config_t* dev_config);

enum garage_state_e mive_garage_get_state(mive_garage_t* garage);

esp_err_t mive_garage_actuate(mive_garage_t* garage);

char* mive_garage_get_state_str(enum garage_state_e state);

#endif // _MIVE_GARAGE_H