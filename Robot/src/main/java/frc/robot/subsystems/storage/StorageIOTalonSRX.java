// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.storage;

import static frc.robot.Constants.*;
import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class StorageIOTalonSRX implements StorageIO {
  private final WPI_TalonSRX beltMotor;
  private final DigitalInput collectorSensor;
  private final DigitalInput shooterSensor;
  private boolean isEnabled;

  public StorageIOTalonSRX() {

    this.isEnabled = false;

    this.beltMotor = new WPI_TalonSRX(StorageConstants.STORAGE_MOTOR_ID);

    // no data is read from the Talon SRX; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.beltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.beltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.collectorSensor = new DigitalInput(StorageConstants.COLLECTOR_SENSOR);

    this.shooterSensor = new DigitalInput(StorageConstants.SHOOTER_SENSOR);
  }

  @Override
  public void updateInputs(StorageIOInputs inputs) {
    inputs.isEnabled = isEnabled;

    inputs.appliedVolts = beltMotor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {beltMotor.getStatorCurrent()};
    inputs.tempCelcius = new double[] {beltMotor.getTemperature()};

    inputs.isShooterSensorUnblocked = this.shooterSensor.get();
    inputs.isCollectorSensorUnblocked = this.collectorSensor.get();
  }

  @Override
  public void setEnabled(boolean enabled) {
    isEnabled = enabled;
  }

  @Override
  public void setMotorPercentage(double percentage) {
    beltMotor.set(ControlMode.PercentOutput, percentage);
  }
}
