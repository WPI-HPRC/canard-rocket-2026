#pragma once

#include "Context.h"

enum StateID {
    PRELAUNCH,
    BOOST,
    CANARDS,
    RECOVERY,
    ABORT,
    NUM_STATES
};

struct StateData {
    long long currentTime;
    long long deltaTime;
    long long loopCount;
    long long startTime;
    long long lastLoopTime;
};

// PRELAUNCH
void prelaunchInit (StateData* data);
StateID prelaunchLoop (StateData* data, Context* ctx);

// BOOST
void boostInit (StateData* data);
StateID boostLoop (StateData* data, Context* ctx);

// COAST
void canardsInit (StateData* data);
StateID canardsLoop (StateData* data, Context* ctx);

// RECOVERY
void recoveryInit(StateData *data);
StateID recoveryLoop (StateData* data, Context* ctx);

// ABORT
void abortInit (StateData* data);
StateID abortLoop (StateData *data, Context *ctx);
