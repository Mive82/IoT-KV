#ifndef _MIVE_EVENTS_H
#define _MIVE_EVENTS_H

#include <stdint.h>
#include "queue.h"

#define EVENT_SET(events, new_event) events.event_type = new_event
#define EVENT_SET_DATA(events, new_event, data) EVENT_SET(events, new_event); events.event_data = data

#define IS_EVENT_SET(events, new_event) events.event_type == new_event
#define EVENT_GET_DATA(events) events.event_data

enum garage_events_e
{
  EVENT_NONE = 0,
  // No data
  EVENT_CLOSED_LIMIT_SWITCH_PRESSED,
  // No data
  EVENT_CLOSED_LIMIT_SWITCH_RELEASED,
  // No data
  EVENT_OPEN_LIMIT_SWITCH_PRESSED,
  // No data
  EVENT_OPEN_LIMIT_SWITCH_RELEASED,
  // Data = key
  EVENT_KEYPAD_NEW_KEY,
  // No data
  EVENT_START_DOOR,
  // No data
  EVENT_STOP_DOOR,
  EVENT_ACTUATE_DOOR,
  EVENT_MAX = EVENT_ACTUATE_DOOR,
};

struct garage_event
{
  uint8_t event_type;
  uint8_t event_data;
};

typedef struct garage_event garage_event_t;

QUEUE_DECLARATION(event_queue, garage_event_t, 10);

#endif // _MIVE_EVENTS_H