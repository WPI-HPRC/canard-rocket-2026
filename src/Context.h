#pragma once

#include <STM32SD.h>
#include <Servo.h>
#include "LoRaE22.h"
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/LIS2MDLTR.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/LSM6.h"
#include "boilerplate/Sensors/Mock/ASM330.h"
#include "boilerplate/Sensors/Mock/LIS2MDLTR.h"
#include "boilerplate/Sensors/Mock/LSM6.h"
#include "boilerplate/Utilities/PwmInput.h"
#include "boilerplate/qmekf-lib/include/split_mekf.h"

struct ASM330Data;
struct LPS22Data;
struct ICMData;
struct MAX10SData;
struct INA219Data;

struct Context {
    File logFile;
    File debugLogFile;
    File ekfLogFile;
    bool sdInitialized;
    bool ekfLooping;

    #if defined(MOCK_SENSORS) && defined(ASM_DATA_FILENAME) && defined(ASM_DATA_RATE)
        MockASM330 asm330;
    #else
        ASM330 asm330;
    #endif

    LSM6 lsm;
    LPS22 baro;

    #if defined(MOCK_SENSORS) && defined(LIS_DATA_FILENAME) && defined(LIS_DATA_RATE)
        MockLIS2MDL mag;
    #else
        LIS2MDL mag;
    #endif

    LIV3F gps;

    LoRaE22 radio;

    // PwmInput encoder1 = PwmInput(ENCODER1_PWM);
    // PwmInput encoder2 = PwmInput(ENCODER2_PWM);
    // PwmInput encoder3 = PwmInput(ENCODER3_PWM);
    // PwmInput encoder4 = PwmInput(ENCODER4_PWM);
    
    SplitStateEstimator estimator;
};
