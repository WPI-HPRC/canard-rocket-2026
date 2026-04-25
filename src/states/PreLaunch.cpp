#include "../State.h"
#include "Arduino.h"
#include "StateMachineConstants.h"
#include "config.h"
#include "debouncer.h"
#include "logging.h"

void prelaunchInit(StateData *data) {}

StateID prelaunchLoop(StateData *data, Context *ctx) {
  static Debouncer accelDebouncer(50);
  const auto acc_vec = ctx->estimator.get_accel_prev();
  // check acceleration in vertical direction is greater than threshold
  if (accelDebouncer.update(abs(acc_vec(0, 0)) > PRELAUNCH_TO_BOOST_MIN_ACCEL, millis())) {
    return BOOST;
  }

  return PRELAUNCH;
}
