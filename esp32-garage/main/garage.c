#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "include/garage.h"

static char* garage_state_str[] = {
    [GARAGE_INVALID] = "GARAGE_INVALID",
    [GARAGE_CLOSED] = "GARAGE_CLOSED",
    [GARAGE_CLOSING_STOPPED] = "GARAGE_STOPPED",
    [GARAGE_OPEN] = "GARAGE_OPEN",
    [GARAGE_OPENING_STOPPED] = "GARAGE_STOPPED",
    [GARAGE_OPENING] = "GARAGE_OPENING",
    [GARAGE_CLOSING] = "GARAGE_CLOSING",   
};

char* mive_garage_get_state_str(enum garage_state_e state)
{
  if(state < 0 || state > GARAGE_STATE_MAX)
  {
    return NULL;
  }

  return garage_state_str[state];
}

esp_err_t mive_garage_init(mive_garage_t* garage, i2c_master_bus_config_t* bus_config, i2c_device_config_t* dev_config)
{
  esp_err_t retval = ESP_OK;

  retval = i2c_new_master_bus(bus_config, &garage->bus_master);

  if(retval != ESP_OK)
  {
    ESP_ERROR_CHECK_WITHOUT_ABORT(retval);
    goto end;
  }

  retval = i2c_master_bus_add_device(garage->bus_master, dev_config, &garage->dev_handle);
  if(retval != ESP_OK)
  {
    ESP_ERROR_CHECK_WITHOUT_ABORT(retval);
    goto delete_master;
  }

  return ESP_OK;

delete_master:
  i2c_del_master_bus(garage->bus_master);

end:
  garage->bus_master = NULL;
  garage->dev_handle = NULL;

  return retval;
}

enum garage_state_e mive_garage_get_state(mive_garage_t* garage)
{
  uint8_t data = 0;
  esp_err_t retval = ESP_OK;
  retval = i2c_master_transmit_receive(garage->dev_handle, &data, 1, &garage->reg.reg, 1, 100);
  if(retval != ESP_OK)
  {
    return GARAGE_INVALID;
  }
  else
  {
    return (enum garage_state_e)garage->reg.s.state;
  }
}

esp_err_t mive_garage_actuate(mive_garage_t* garage)
{
  uint8_t data[2] = {0};
  esp_err_t retval = ESP_OK;
  union garage_i2c_reg reg = {
    .s.command = 1
  };

  data[1] = reg.reg;

  retval = i2c_master_transmit(garage->dev_handle, data, 2, 100);

  return retval;
}
