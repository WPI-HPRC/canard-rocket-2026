#include "States.h"
#include "../State.h"


StateInitFunc initFuncs[NUM_STATES] = {};
StateLoopFunc loopFuncs[NUM_STATES] = {};

void initStateMap() {
  initFuncs[PRELAUNCH] = &prelaunchInit;
  initFuncs[BOOST] = &boostInit;
  initFuncs[CANARDS] = &canardsInit;
  initFuncs[RECOVERY] = &recoveryInit;
  initFuncs[ABORT] = &abortInit;

  loopFuncs[PRELAUNCH] = &prelaunchLoop;
  loopFuncs[BOOST] = &boostLoop;
  loopFuncs[CANARDS] = &canardsLoop;
  loopFuncs[RECOVERY] = &recoveryLoop;
  loopFuncs[ABORT] = &abortLoop;
}
