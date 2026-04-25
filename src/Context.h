#pragma once

#include <STM32SD.h>
#include <Servo.h>
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/LIS2MDLTR.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/LSM6.h"
#include "boilerplate/Sensors/Mock/ASM330.h"
#include "boilerplate/Sensors/Mock/LIS2MDLTR.h"
#include "boilerplate/Sensors/Mock/LSM6.h"
#include "boilerplate/qmekf-lib/include/split_mekf.h"

struct ASM330Data;
struct LPS22Data;
struct ICMData;
struct MAX10SData;
struct INA219Data;

struct Context {
    File logFile;
    File errorLogFile;
    File fixedRateLogFile;
    File ekfLogFile;
    bool sdInitialized;
    bool ekfLooping;

    MockASM330 asm330;
    // ASM330 asm330;
    LSM6 lsm;
    LPS22 baro;
    MockLIS2MDL mag;
    // LIS2MDL mag;
    LIV3F gps;

    // LoRaE22 radio;
    
    SplitStateEstimator estimator;
};
