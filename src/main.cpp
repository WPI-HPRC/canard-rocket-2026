#include "config.h"

#include <Arduino.h>

#include "Context.h"

#include "Packet_generated.h"
#include "Sensors_generated.h"
#include "RemoteControl_generated.h"
#include "RocketCanardsTelemetryPacket_generated.h"
#include "flatbuffers/buffer.h"

#ifdef __has_include
#if __has_include("states/States.h")
#include "State.h"
#include "states/States.h"
#else
#include "template_states/States.h"
#define TEMPLATE_STATES_OVERRIDE
#include "State.h"
#endif
#else
#warning No __has_include, falling back to template_states
#include "template_states/States.h"
#endif

#include "boilerplate/Sensors/SensorManager/SensorManager.h"

#include "logging.h"

#include "LoRaE22.h"
#include "RadioConfig.h"
#include <Servo.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

// #define PRINT_SENSORS

SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

Context ctx{
    #if defined(MOCK_SENSORS) && defined(ASM_DATA_FILENAME) && defined(ASM_DATA_RATE)
      .asm330 = MockASM330(ASM_DATA_FILENAME, ASM_DATA_RATE),
    #else
      .asm330 = ASM330(&SENSORS_SPI, SENSORS_ASM_CS),
    #endif
    .lsm = LSM6(&SENSORS_SPI, SENSORS_LSM_CS),
    .baro = LPS22(&SENSORS_SPI, SENSORS_LPS_CS),
    #if defined(MOCK_SENSORS) && defined(LIS_DATA_FILENAME) && defined(LIS_DATA_RATE)
      .mag = MockLIS2MDL(LIS_DATA_FILENAME, LIS_DATA_RATE),
    #else
      .mag = LIS2MDL(&SENSORS_SPI, SENSORS_LIS_CS),
    #endif
    .gps = LIV3F(GPS_SERIAL),
    .radio = LoRaE22(&RADIO_SERIAL, RADIO_M0, RADIO_M1, RADIO_AUX, "KV0R"),
};

SensorManager mgr{
    millis, ctx.asm330, ctx.lsm, ctx.baro, ctx.mag, ctx.gps,
};

StateID currentState;
StateData data;

void handleCommand(hprc::Command cmd) {
  switch (cmd) {
  case hprc::Command_ArmFlight:
    break;
  case hprc::Command_DeArmFlight:
    break;
  case hprc::Command_Reset:
    break;
  case hprc::Command_RemoteStartOn:
    break;
  case hprc::Command_RemoteStartOff:
    break;
  case hprc::Command_CanardsTest:
    break;
  case hprc::Command_CanardsReset:
    break;
  case hprc::Command_CanardsDisable:
    break;
  case hprc::Command_CanardsEnable:
    break;
  case hprc::Command_StartEstimator:
    break;
  case hprc::Command_Abort:
    break;
  }
}

bool changeSerialPortConfig(RadioConfigTypes::SerialSpeeds baudRate,
                            RadioConfigTypes::ParityConfig parity) {
  // this is safe to call even when the port is not open.
  RADIO_SERIAL.end();

  uint32_t baud = 0;
  uint16_t parityConfig = 0;

  // the radio's baud rates don't follow any pattern over the entire range, so
  // ugly switch statement it is
  switch (baudRate) {
  case RadioConfigTypes::SerialSpeeds::BAUD_1200:
    baud = 1200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_2400:
    baud = 2400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_4800:
    baud = 4800;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_9600:
    baud = 9600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_19200:
    baud = 19200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_38400:
    baud = 38400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_57600:
    baud = 57600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_115200:
    baud = 115200;
    break;
  };
  // this is just easier
  switch (parity) {
  case RadioConfigTypes::ParityConfig::Parity_8N1:
    parityConfig = SERIAL_8N1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8E1:
    parityConfig = SERIAL_8E1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8O1:
    parityConfig = SERIAL_8O1;
    break;
  };

  RADIO_SERIAL.begin(baud, parityConfig);

  return true;
}

void radioInit() {
  // build our config
  RadioConfig config;
  config.address = ADDRESS;
  config.networkId = NETWORKID;
  config.encryptionKey = ENCRYPTIONKEY;
  config.parityConfig = PARITYCONFIG;
  config.serialSpeed = SERIALSPEED;
  config.airDataRate = AIRDATARATE;
  config.packetSize = PACKETSIZE;
  config.worMode = WORMODE;
  config.worPeriod = WORPERIOD;
  config.relayMode = RELAYMODE;
  config.destination = DESTINATIONMODE;
  config.txPower = dBm33;
  config.ambientRSSIEnabled = AMBIENTRSSI;
  config.rssiReadingsEnabled = RSSIREADINGS;
  config.listenBeforeTxEnable = LISTENBEFORETX;
  ctx.radio.setConfig(config);
  ctx.radio.setFrequency(FREQUENCY);

  ctx.radio.changeSerialPortCallback(changeSerialPortConfig);
  ctx.radio.setTimeout(2000);

#ifdef RADIO_DEBUG
  // uint8_t configBuffer[9];
  // Serial.println("trying to build config buffer");
  // ctx.radio.buildConfigBuffer(configBuffer);
  // Serial.print("desired config registers: ");
  // for (size_t i; i < sizeof(configBuffer); i++) {
  //   Serial.printf("%0X ", configBuffer[i]);
  // }
  // Serial.println();
#endif

  int8_t code = ctx.radio.init(3);
  ctx.radio.setMode(RadioMode::Normal);

#ifdef RADIO_DEBUG
  Serial.println("done initing");
  Serial.println(code);
#endif
}
float ekfStartTime;
void radioLoop() {
  // static flatbuffers::FlatBufferBuilder builder;
  // static uint8_t rxBuff[1024];
  // static ssize_t lastCmdNum = -1;
  static uint32_t lastRadioSendTime = 0;

  ctx.radio.update();

  if (millis() - lastRadioSendTime >= 50) {
    lastRadioSendTime = millis();
    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    RADIO_SERIAL.print("KV0R");

    RADIO_SERIAL.print("Quaternion: ");
    auto q = ctx.estimator.get_quat_ned();
    RADIO_SERIAL.print(q(0)); RADIO_SERIAL.print(", "); 
    RADIO_SERIAL.print(q(1)); RADIO_SERIAL.print(", ");
    RADIO_SERIAL.print(q(2)); RADIO_SERIAL.print(", ");
    RADIO_SERIAL.println(q(0));

    BLA::Matrix<3, 1> gb = ctx.estimator.get_gyro_bias();
    RADIO_SERIAL.print("Gyro Bias: ");
    RADIO_SERIAL.print(gb(0, 0), 8);
    RADIO_SERIAL.print(gb(1, 0), 8);
    RADIO_SERIAL.println(gb(2, 0), 8);

    // hprc::Sensors sensors = hprc::Sensors(
    //     asm330_desc.data.accel0, asm330_desc.data.accel1,
    //     asm330_desc.data.accel2, asm330_desc.data.gyr0, asm330_desc.data.gyr1,
    //     asm330_desc.data.gyr2, lsm6_desc.data.accel0, lsm6_desc.data.accel1,
    //     lsm6_desc.data.accel2, lsm6_desc.data.gyr0, lsm6_desc.data.gyr1,
    //     lsm6_desc.data.gyr2, mag_desc.data.mag0, mag_desc.data.mag1,
    //     mag_desc.data.mag2, baro_desc.data.pressure, baro_desc.data.temp);

    // builder.Clear();

    // flatbuffers::Offset<hprc::RocketCanardsTelemetryPacket> packetInner =
    //     hprc::CreateRocketCanardsTelemetryPacket(builder, nullptr,
    //                                              hprc::States_Start, &sensors);

    // flatbuffers::Offset<hprc::Packet> packet = hprc::CreatePacket(
    //     builder, hprc::PacketUnion_RocketCanardsTelemetryPacket,
    //     packetInner.Union());

    // builder.Finish(packet);

    // ctx.radio.sendMessage(builder.GetBufferPointer(), builder.GetSize());
  }

  // if (ctx.radio.hasMessage()) {
  //   uint8_t bytesRead;
  //   ctx.radio.getMessage(rxBuff, 1024, bytesRead);

  //   const hprc::Packet *packet = hprc::GetPacket(rxBuff);
  //   if (packet->packet_type() != hprc::PacketUnion_RemoteControl) {
  //     Log.errorln("Recieved wrong packet type: %d", packet->packet_type());
  //   }
  //   const auto command = static_cast<const hprc::RemoteControlCommand *>(
  //       packet->packet_as_RemoteControl());
  //   if (lastCmdNum >= command->command_number()) {
  //     Log.errorln("Received old packet number: %d (expecting > %d)",
  //                 command->command_number(), lastCmdNum);
  //   }
  //   lastCmdNum = command->command_number();

  //   Log.traceln("Received cmd: %d (#%d)", command->command(), command->command_number());

  //   handleCommand(command->command());
  // }

 

  if(RADIO_SERIAL.available() > 0){
    uint8_t command = RADIO_SERIAL.read();
    if(command == 's'){
        ekfStartTime = millis() / 1000.0f;
        ctx.ekfLooping = true;
    }
    else{
      // kill canards
    }
  }
}

void initCanardServos() {
  Log.infoln("Beginning encoders");

  pinMode(CANARD_POWER_ENABLE, OUTPUT);
  pinMode(LEVEL_SHIFT_ENABLE, OUTPUT);
  
  digitalWrite(CANARD_POWER_ENABLE, HIGH);
  digitalWrite(LEVEL_SHIFT_ENABLE, HIGH);
  
  if(!ctx.encoder1.begin()) {
    Log.infoln("Encoder 1 begin");
  } else {
    Log.infoln("Encoder 1 FAILED to begin");
  }
  
  if(!ctx.encoder2.begin()) {
    Log.infoln("Encoder 2 begin");
  } else {
    Log.infoln("Encoder 2 FAILED to begin");
  }
  if(!ctx.encoder3.begin()) {
    Log.infoln("Encoder 3 begin");
  } else {
    Log.infoln("Encoder 3 FAILED to begin");
  }
  if(!ctx.encoder4.begin()) {
    Log.infoln("Encoder 4 begin");
  } else {
    Log.infoln("Encoder 4 FAILED to begin");
  }

  uint8_t status = ctx.c1.attach(SERVO1_PWM);
  Serial.printf("Attached to Servo 1, STATUS %02X\n", status);

  status = ctx.c2.attach(SERVO2_PWM);                
  Serial.printf("Attached to Servo 2, STATUS %02X\n", status);

  status = ctx.c3.attach(SERVO3_PWM);    
  Serial.printf("Attached to Servo 3, STATUS %02X\n", status);
                                           
  status = ctx.c4.attach(SERVO4_PWM);
  Serial.printf("Attached to Servo 4, STATUS %02X\n", status);

  Log.infoln("Initialized Canards Encoders and Servos");
}

void initStateData(StateData *data) {
  data->startTime = millis();
  data->currentTime = 0;
  data->deltaTime = 0;
  data->lastLoopTime = 0;
  data->loopCount = 0;
}

void updateStateData(StateData *data) {
  long long now = millis();
  data->currentTime = now - data->startTime;
  data->deltaTime = now - data->lastLoopTime;
  data->lastLoopTime = now;
  data->loopCount++;
}

void canardLoop() {
  static unsigned long last_print = 0;
  static int loop_count = 0;

  if (millis() - last_print > 500) {
    int value = loop_count % 2 == 0 ? 90 : 0;

    /*
    BLA::Matrix<3,1> rpy =  ctx.estimator.get_rpy_ned();
    float p = rpy(1, 1);
    float value = 90 - p;
    */

    ctx.c1.write(value);
    ctx.c2.write(value);
    ctx.c3.write(value);
    ctx.c4.write(value);
    
    loop_count++;
    last_print = millis();

    digitalWrite(LED_RED, !digitalRead(LED_RED));

    // TODO: Fix!
    // float dutyCycle = ctx.encoder2.getDutyCycle();
    // float freq      = ctx.encoder2.getFrequency();

    // Serial.print("Encoder 2 duty cycle: "); Serial.println(dutyCycle, 2);
    // Serial.print("Encoder 2 frequency: "); Serial.println(freq, 2);
    // Serial.printf("Read Encoder 1\n");
  }
}

void sensorsSetup() {
  Log.infoln("Starting MARS board initialization...");
  SENSORS_SPI.begin();

  mgr.sensorInit();

  Log.infoln("\n=== Sensor Initialization Summary ===");
  Log.infoln("Total sensors: %d", mgr.count());
}

void sensorLoop() {
  static unsigned long last_print = 0;
  static int loop_count = 0;

  // Update all sensors through manager
  mgr.loop();

  /*
  if (currentState >= PRELAUNCH) {
      return;
  }
  */

  // manager is not being used here to get data
  if (millis() - last_print > 200) {
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    last_print = millis();
    loop_count++;

    #ifdef PRINT_SENSORS
    Log.infoln("=== Loop %d ===", loop_count);

    // DIRECT ACCESS to sensor data - this is guaranteed to work
    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    bool has_data = false;
    // Print LSM6 data
    if (lsm6_desc.getLastUpdated() > 0) {
      Log.infoln("LSM6DSO - Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                 lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                 lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("LSM6DSO: No data (timestamp = 0)");
    }

    // Print ASM330 data
    if (asm330_desc.getLastUpdated() > 0) {
      Log.infoln("ASM330- Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 asm330_desc.data.accel0, asm330_desc.data.accel1,
                 asm330_desc.data.accel2, asm330_desc.data.gyr0,
                 asm330_desc.data.gyr1, asm330_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("ASM330: No data (timestamp = 0)");
    }
    // Print LPS22 data
    if (baro_desc.getLastUpdated() > 0) {
      Log.infoln("LPS22 - Pressure: %F hPa, Temp: %F C",
                 baro_desc.data.pressure, baro_desc.data.temp);
      has_data = true;
    } else {
      Log.warningln("LPS22: No data (timestamp = 0)");
    }

    if (mag_desc.getLastUpdated() > 0) {
      Log.infoln("LIS2MDL - Mag: %F, %F, %F", mag_desc.data.mag0,
                 mag_desc.data.mag1, mag_desc.data.mag2);
      has_data = true;
    } else {
      Log.warningln("ICM20948: No data (timestamp = 0)");
    }

    if (gps_desc.getLastUpdated() > 0) {
      Log.infoln("LIV3F - Lat, Lon, Alt: %F, %F, %F | Satellites - %d",
                 gps_desc.data.lat, gps_desc.data.lon, gps_desc.data.alt,
                 gps_desc.data.satellites);
      has_data = true;
    } else {
      Log.warningln("LIV3F: No data (timestamp = 0)");
    }

    // Log.infoln("Encoder1: %F%% @ %F Hz", ctx.encoder1.getDutyCycle());
    // Log.infoln("Encoder2: %F%% @ %F Hz", ctx.encoder2.getDutyCycle());
    // Log.infoln("Encoder3: %F%% @ %F Hz", ctx.encoder3.getDutyCycle());
    // Log.infoln("Encoder4: %F%% @ %F Hz", ctx.encoder4.getDutyCycle());

    Log.infoln("======================\n");
    #endif
  }
}

void setup() {
  currentState = PRELAUNCH;
  data = {};

  initStateMap();

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_RED, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  delay(200);

  ctx.sdInitialized = initializeLogging(&ctx);

#if DEBUG_MODE == 0
  Log.begin(LOG_LEVEL_INFO, &ctx.debugLogFile);
#elif DEBUG_MODE == 1
  Log.begin(LOG_LEVEL_INFO, &Serial);
#else
  Log.begin(LOG_LEVEL_TRACE, &Serial);
#endif
  Log.setPrefix([](Print *p, int level) { p->printf("[ %d ] ", millis()); });


  // NOTE: Run initialization on the first state
  initStateData(&data);
  (*initFuncs[currentState])(&data);
  sensorsSetup();

  radioInit();

  initCanardServos();

  ctx.ekfLooping = false;

  ctx.estimator.init(millis() / 1000.0f, {0, 0, 0}, {0, 0, 0});

  digitalWrite(LED_GREEN, HIGH);
  Log.infoln("=== Starting main loop ===\n");
}

void ekfLoop(Context *ctx) {
  static uint32_t last_accel_time = 0;
  static uint32_t last_mag_time = 0;
  static uint32_t last_gps_time = 0;
  static uint32_t last_baro_time = 0;

  static BLA::Matrix<3, 1> accel = {0, 0, 0};
  static BLA::Matrix<3, 1> gyro = {0, 0, 0};
  static BLA::Matrix<3, 1> mag = {0, 0, 0};
  static BLA::Matrix<1, 1> baro = {0};

  static int state = 0;
  static BLA::Matrix<6, 1> lastCalcTimes = {0, 0, 0, 0, 0, 0};
  static BLA::Matrix<6, 1> runRates = {0.001, 0.03, 0.03, 0.5, 1, 1};

  static BLA::Matrix<3, 1> initial_roll_axis_vec = {0, 0, 0};
  static BLA::Matrix<3, 1> new_roll_axis_vec = {0, 0, 0};
  static BLA::Matrix<3, 1> roll_axis = {1, 0, 0};

  static BLA::Matrix<3, 1> initial_pitch_axis_vec = {0, 0, 0};
  static BLA::Matrix<3, 1> pitch_axis = {0, 1, 0};
  static BLA::Matrix<3, 1> new_pitch_axis_vec = {0, 0, 0};

  static BLA::Matrix<3, 1> initial_yaw_axis_vec = {0, 0, 0};
  static BLA::Matrix<3, 1> yaw_axis = {0, 0, 1};
  static BLA::Matrix<3, 1> new_yaw_axis_vec = {0, 0, 0};

  float t = millis() / 1000.0f - ekfStartTime;

  const auto &asm330_desc = ctx->asm330.get_descriptor();
  const auto &baro_desc = ctx->baro.get_descriptor();
  const auto &mag_desc = ctx->mag.get_descriptor();
  const auto &gps_desc = ctx->gps.get_descriptor();

  accel = {asm330_desc.data.accel0, asm330_desc.data.accel1,
           asm330_desc.data.accel2};
  gyro = {asm330_desc.data.gyr0, asm330_desc.data.gyr1, asm330_desc.data.gyr2};
  mag = {mag_desc.data.mag0, mag_desc.data.mag1, mag_desc.data.mag2};
  baro = {baro_desc.data.pressure};

  if (state == 0) {
    if (t > 5.0f) {
      state = 1;
    }
    ctx->estimator.computeInitialOrientation(ctx->estimator.reorient_asm(accel),
                                             ctx->estimator.reorient_lis(mag),
                                             t);

    lastCalcTimes = {t, t, t, t, t, t};

    auto R0 = quat2DCM(ctx->estimator.get_quat_ned());

    initial_roll_axis_vec = R0 * roll_axis;
    initial_pitch_axis_vec = R0 * pitch_axis;
    initial_yaw_axis_vec = R0 * yaw_axis;

  } else if (state == 1) {
    if (t > 15.0f) {
      state = 2;
      auto R0 = quat2DCM(ctx->estimator.get_quat_ned());

      initial_roll_axis_vec = R0 * roll_axis;
      initial_pitch_axis_vec = R0 * pitch_axis;
      initial_yaw_axis_vec = R0 * yaw_axis;
    }
    if (t - lastCalcTimes(0, 0) >= runRates(0, 0)) {

      ctx->estimator.fastGyroProp(ctx->estimator.reorient_asm(gyro), t);
      ctx->estimator.AttekfPredict(t);
      ctx->estimator.runAccelMagUpdate(ctx->estimator.reorient_asm(accel),
                                       ctx->estimator.reorient_lis(mag), t);
      // ctx->estimator.runMagUpdate(ctx->estimator.reorient_lis(mag), t);

      lastCalcTimes(0, 0) = t;
      // SerialUSB.println("GYRO PROPPING");
    }
  } else if (state == 2) {
    if (t - lastCalcTimes(0, 0) >= runRates(0, 0)) {

      ctx->estimator.fastGyroProp(ctx->estimator.reorient_asm(gyro), t);
      // ctx->estimator.AttekfPredict(t);
      // ctx->estimator.runAccelMagUpdate(ctx->estimator.reorient_asm(accel),
      // ctx->estimator.reorient_lis(mag), t , 100.0);
      lastCalcTimes(0, 0) = t;
    }
  }

  auto quat = ctx->estimator.get_quat_ned();
  auto gb = ctx->estimator.get_gyro_bias();

  ctx->errorLogFile.print(millis());
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(quat(0), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(quat(1), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(quat(2), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(quat(3), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(gb(0), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(gb(1), 8);
  ctx->errorLogFile.print(',');
  ctx->errorLogFile.print(gb(2), 8);
  ctx->errorLogFile.println();

  // static uint32_t lastFlushTime = 0;
  // if (millis() - lastFlushTime >= 1000) {
  //   lastFlushTime = millis();
  //   ctx->errorLogFile.flush();
  // }

  // auto q = ctx->estimator.get_quat_ned();

  // SerialUSB.print("Q: ");
  // SerialUSB.print(q(0)); SerialUSB.print(", ");
  // SerialUSB.print(q(1)); SerialUSB.print(", ");
  // SerialUSB.print(q(2)); SerialUSB.print(", ");
  // SerialUSB.println(q(3));

  // Serial.print("Time: ");
  // Serial.print(t);
  // Serial.print(". Covariance: ");
  // BLA::Matrix<12, 1> P = ctx->estimator.getAttPDiag();
  // Serial.print(P(0, 0)); Serial.print(", "); Serial.print(P(1, 0));
  // Serial.print(", "); Serial.print(P(2, 0));

  // auto R = quat2DCM(ctx->estimator.get_quat_ned());

  // new_roll_axis_vec = R * roll_axis;
  // new_pitch_axis_vec = R * pitch_axis;
  // new_yaw_axis_vec = R * yaw_axis;

  // float roll_diff = acos(vecDot(initial_roll_axis_vec, new_roll_axis_vec));
  // float pitch_diff = acos(vecDot(initial_pitch_axis_vec,
  // new_pitch_axis_vec)); float yaw_diff = acos(vecDot(initial_yaw_axis_vec,
  // new_yaw_axis_vec));

  // Serial.print(". Roll: ");
  // Serial.print(roll_diff);

  // Serial.print(" Pitch: ");
  // Serial.print(pitch_diff);

  // Serial.print(" Yaw: ");
  // Serial.print(yaw_diff);

  // Serial.print(". Gyro bias: ");
  // BLA::Matrix<3, 1> gb = ctx->estimator.get_gyro_bias();
  // Serial.print(gb(0, 0), 7);
  // Serial.print(", ");
  // Serial.print(gb(1, 0), 7);
  // Serial.print(", ");
  // Serial.print(gb(2, 0), 7);
  // Serial.println();
}

void loop() {

  updateStateData(&data);
  StateID newState = (*loopFuncs[currentState])(&data, &ctx);

  if (currentState != newState) {
    initStateData(&data);
    (*initFuncs[newState])(&data);
    currentState = newState;
    Log.infoln("Transitioned to %d", newState);
  }

  sensorLoop();

  // radioLoop();

  if (ctx.ekfLooping) {
    ekfLoop(&ctx);
  }

  canardLoop();

  loggingLoop(&ctx);
}
