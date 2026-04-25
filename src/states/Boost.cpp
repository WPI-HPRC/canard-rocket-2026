#include "../State.h"
#include "StateMachineConstants.h"
#include "debouncer.h"

void boostInit(StateData *data) {}

StateID boostLoop(StateData *data, Context *ctx) {
  /*
  - Poll acceleration data from ctx
  - Check acceleration to detect coast stage
  - Check if maximum boost time is exceeded
  - Check if need to abort
  - Update sensor data and ctx for next iteration?
  */
  // Need 50ms of sustained acceleration near 0
  static Debouncer accelDebouncer(50);

  // XXX: Someone should check that this is actually the upwards direction, and
  // this is a reasonable way to determine that we have sustained upwards
  // acceleration
  const auto acc_vec = ctx->estimator.get_accel_prev();
  if (accelDebouncer.update(abs(acc_vec(0, 0)) < BOOST_TO_COST_ACCEL_EPSILON,
                            millis()) ||
      data->currentTime > MAX_MOTOR_BURN_TIME) {
    // check that acceleration up is less than threshold, or
    // current time > 2000
    return CANARDS;
  }

  return BOOST;
}
