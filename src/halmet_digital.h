#ifndef __SRC_HALMET_DIGITAL_H__
#define __SRC_HALMET_DIGITAL_H__

#include "sensesp/sensors/sensor.h"

using namespace sensesp;

FloatProducer* ConnectTachoSender(int pin, String name);
BoolProducer* ConnectAlarmSender(int pin, String name, bool inverted = false, bool enable_signalk_output = true);

#endif
