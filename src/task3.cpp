#include <functions.h>

volatile unsigned long usedUpLoopTimeTask3;



void task3_setup() {
  Wire.begin();
  Wire.setClock(1000000);

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

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

  prevAuxChannel = channel[2];
  usedUpLoopTimeTask3 = 0;
}


void task3_loop() {
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
}


void task3(void *parameter) {
  task3_setup();
  unsigned long loopTimer;  
  loopTimer = micros();
  for (;;) {
    task3_loop();
    usedUpLoopTimeTask3 = micros() - loopTimer;
    //Serial.println(usedUpLoopTimeTask3);
    while(micros() - loopTimer < LOOP_TIME_TASK3) {
      vTaskDelay(1);
    };    
    loopTimer = micros();
  }
}
