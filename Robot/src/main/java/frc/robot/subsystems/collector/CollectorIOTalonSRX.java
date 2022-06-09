// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import static frc.robot.Constants.*;
import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class CollectorIOTalonSRX implements CollectorIO {
  private final WPI_TalonSRX collectorMotor;
  private final Solenoid collectorPiston;
  private boolean isEnabled;

  public CollectorIOTalonSRX() {

    collectorMotor = new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_ID);
    collectorMotor.setInverted(true);

    // no data is read from the Talon SRX; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    collectorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    collectorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    collectorPiston =
        new Solenoid(
            CollectorConstants.PEUNAMATICS_HUB_CAN_ID,
            PneumaticsModuleType.REVPH,
            CollectorConstants.COLLECTOR_SOLENOID_CHANNEL);
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    inputs.isEnabled = isEnabled;

    inputs.appliedVolts = collectorMotor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {collectorMotor.getStatorCurrent()};
    inputs.tempCelcius = new double[] {collectorMotor.getTemperature()};
  }

  @Override
  public void setMotorPercentage(double percentage) {
    collectorMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setExtended(boolean extended) {
    collectorPiston.set(extended);
  }
}
