#include <functions.h>


unsigned long loopTimer, voltageTimer, ledTimer;
int lowVoltageAlarmCount;


void setup2() {
  Wire.begin();
  Wire.setClock(1000000);
  
  nrOfLEDStrips = getNrOfLEDStrips();

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  signal_micros_timer_value = 0;
  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

  hasCurrentSensor = getHasCurrentSensor();
  drivingMode = getDefaultDrivingMode();

  pinMode(RECEIVER_STEER_PIN, INPUT);
  pinMode(RECEIVER_THROTTLE_PIN, INPUT);
  pinMode(RECEIVER_SPEED_MODE_PIN, INPUT);

  pinMode(VOLTAGE_SENSOR_PIN, ANALOG);
  pinMode(CURRENT_SENSOR_PIN, ANALOG);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // pwm
  ledcSetup(MOTOR_STEERING_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(MOTOR_LB_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_RB_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_LF_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_RF_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);

  ledcAttachPin(MOTOR_STEERING_PIN,MOTOR_STEERING_PWM_CHANNEL);
  ledcAttachPin(MOTOR_LB_PIN, MOTOR_LB_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RB_PIN, MOTOR_RB_PWM_CHANNEL);
  ledcAttachPin(MOTOR_LF_PIN, MOTOR_LF_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RF_PIN, MOTOR_RF_PWM_CHANNEL);

  writeServoPWM(MOTOR_STEERING_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_LB_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_RB_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_LF_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_RF_PWM_CHANNEL, 0);

  initLEDs();  

  voltage = readVoltage();
  lowVoltageAlarmCount = 0;
  
  attachInterrupt(RECEIVER_STEER_PIN, handler_channel_1, CHANGE);
  attachInterrupt(RECEIVER_THROTTLE_PIN, handler_channel_2, CHANGE);
  attachInterrupt(RECEIVER_SPEED_MODE_PIN, handler_channel_3, CHANGE);
  
  initValues();

  prevAuxChannel = channel[2];
  prev_signal_detected = signal_detected;
  
  usedUpLoopTime = 0;
  voltageTimer = millis();
  ledTimer = millis();
  loopTimer = micros();    
}


void loop2() {
  unsigned long millisTimer = millis();

  if (signal_detected) {
    if (!prev_signal_detected) {
      playSignalDetected();
    }
  } else {
    if (prev_signal_detected) {
      playSignalLost();
    }
  }
  prev_signal_detected = signal_detected;  

  int yawChannel = fixChannelDirection(getExpo(channel[0], steerExpoFactor), yawChannelReversed);
  int throttleChannel = fixChannelDirection(channel[1], throttleChannelReversed);
  int auxChannel = fixChannelDirection(channel[2], auxChannelReversed);

  if (((auxChannel > 1750) && (prevAuxChannel < 1250)) || ((auxChannel < 1250) && (prevAuxChannel > 1750))) {
    if (isValidSignalValue(auxChannel)) {
      Serial.println(auxChannel);
      prevAuxChannel = auxChannel;
      incDrivingMode();
      playDrivingMode();
      Serial.print("DrivingMode: ");
      Serial.println(drivingMode);
    }
  }

  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  calcAngles();

  gyro_roll_input = calcDegreesPerSecond(gyro_roll_input, gyro_roll.get());
  gyro_pitch_input = calcDegreesPerSecond(gyro_pitch_input, gyro_pitch.get());
  gyro_yaw_input = calcDegreesPerSecond(gyro_yaw_input, gyro_yaw.get());

  pid_yaw_setpoint = calcPidSetPoint(yawChannel, 0.0);

  yawOutputPID.calc(gyro_yaw_input, pid_yaw_setpoint);

  calcMotorValues(yawChannel, throttleChannel);

  writeServoPWM(MOTOR_STEERING_PWM_CHANNEL, steerServo + steerServoCenterOffset);

  if (signal_detected && isArmed()) {  
    writeEscPWM(MOTOR_LB_PWM_CHANNEL, leftEscs);
    writeEscPWM(MOTOR_RB_PWM_CHANNEL, rightEscs);
    writeEscPWM(MOTOR_LF_PWM_CHANNEL, leftEscs);
    writeEscPWM(MOTOR_RF_PWM_CHANNEL, rightEscs);
  } else {
    writeEscPWM(MOTOR_LB_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_RB_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_LF_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
    writeEscPWM(MOTOR_RF_PWM_CHANNEL, MID_CHANNEL + speedEscCenterOffset);
  }

  // check battery voltage once per second
  if ((millisTimer - voltageTimer) > 1000) {
    voltageTimer = millisTimer;
//    Serial.println(batteryVoltage);         
    if (voltage < LOW_VOLTAGE_ALARM) {
//      Serial.println("lowVoltageAlarmCount++");       
      lowVoltageAlarmCount++;
      if (lowVoltageAlarmCount >= 10) {
//        Serial.println("lowVoltageAlarm!!"); 
        playLowVoltageAlarm(); 
      }
    } else {
      lowVoltageAlarmCount = 0;
    }
  }    

  // update LEDs
  if ((millisTimer - ledTimer) > 250) {
    ledTimer = millisTimer;
    updateLEDs();
  }

  rtttl::play();

  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  usedUpLoopTime = micros() - loopTimer;
  /*
  Serial.print(usedUpLoopTime);
  Serial.println();
  */
  while(micros() - loopTimer < LOOP_TIME) {
    vTaskDelay(1);
  };
  loopTimer = micros();           
}


void runOnCore2(void *parameter) {
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());
  setup2();
  for (;;) {
    loop2();
  }
}
