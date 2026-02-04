#ifndef _MIVE_EVENTS_H
#define _MIVE_EVENTS_H

enum mive_event_e
{
  MIVE_EVENT_NONE = 0,
  MIVE_EVENT_GET_GARAGE_INFO,
  MIVE_EVENT_SEND_GARAGE_INFO,
  MIVE_EVENT_SEND_AUTH_STATE,
  MIVE_EVENT_START_GARAGE,
  MIVE_EVENT_MEASURE_DISTANCE,
  MIVE_EVENT_REGISTER_CARD,
  MIVE_EVENT_MQTT_CONNECTED,
  MIVE_EVENT_SAVE_UUID,
  MIVE_EVENT_RESET_GARAGE_SWITCH,
};

struct mive_event_send_garage_info
{
  uint8_t garage_state;
};

struct mive_event_send_auth_state
{
  uint8_t auth_state;
};

struct mive_event_s
{
  unsigned int event_type;
  union {
    struct mive_event_send_garage_info send_garage_info;
    struct mive_event_send_auth_state send_auth_state;
  } event_data;
};

typedef struct mive_event_s mive_event_t;

#endif // _MIVE_EVENTS_H