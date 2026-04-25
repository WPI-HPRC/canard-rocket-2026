#include "../State.h"

// TODO: tune these microsecond values against actual canard angles
#define CANARD_PLUS_5_DEG  1600  // placeholder for +5 degrees
#define ZERO 1500
#define CANARD_MINUS_5_DEG 1400  // placeholder for -5 degrees  

void canardsInit (StateData* data) { }

StateID canardsLoop (StateData* data, Context* ctx) { 
    if (data->currentTime < 5000) {
        ctx->c1.writeMicroseconds(CANARD_PLUS_5_DEG);
        ctx->c2.writeMicroseconds(CANARD_PLUS_5_DEG);
        ctx->c3.writeMicroseconds(CANARD_MINUS_5_DEG);
        ctx->c4.writeMicroseconds(CANARD_MINUS_5_DEG);
    } else {
        ctx->c1.writeMicroseconds(ZERO);
        ctx->c2.writeMicroseconds(ZERO);
        ctx->c3.writeMicroseconds(ZERO);
        ctx->c4.writeMicroseconds(ZERO);
    }

    return CANARDS;
}
