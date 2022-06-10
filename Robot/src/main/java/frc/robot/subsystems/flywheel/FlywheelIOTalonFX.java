// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static frc.robot.Constants.FlywheelConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final WPI_TalonFX leftFlywheelMotor;
  private final WPI_TalonFX rightFlywheelMotor;

  public FlywheelIOTalonFX() {

    leftFlywheelMotor = new WPI_TalonFX(LEFT_FLYWHEELMOTOR_CANID);
    rightFlywheelMotor = new WPI_TalonFX(RIGHT_FLYWHEELMOTOR_CANID);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behaviour */
    rightFlywheelMotor.configFactoryDefault();
    leftFlywheelMotor.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    /* Disable all motors */
    this.rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);
    this.leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);

    /* Set neutral modes */
    this.leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
    this.rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

    this.leftFlywheelMotor.follow(this.rightFlywheelMotor);

    /* Configure output */
    this.rightFlywheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
    this.leftFlywheelMotor.setInverted(TalonFXInvertType.Clockwise);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     *
     * this.rightFlywheelMotor.setSensorPhase(true)
     */

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    rightConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    /* FPID for velocity */
    rightConfig.slot0.kF = VELOCITY_PID_F;
    rightConfig.slot0.kP = VELOCITY_PID_P;
    rightConfig.slot0.kI = VELOCITY_PID_I;
    rightConfig.slot0.kD = VELOCITY_PID_D;
    rightConfig.slot0.integralZone = VELOCITY_PID_I_ZONE;
    rightConfig.slot0.closedLoopPeakOutput = VELOCITY_PID_PEAK_OUTPUT;

    /* Config the neutral deadband. */
    rightConfig.neutralDeadband = 0.001;

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if sensor updates are
     * too slow - sensor deltas are very small per update, so derivative error never gets large
     * enough to be useful. - sensor movement is very slow causing the derivative error to be near
     * zero.
     */
    int closedLoopTimeMs = 1;
    rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    /* APPLY the config settings */
    this.rightFlywheelMotor.configAllSettings(rightConfig);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.leftFlywheelMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.leftFlywheelMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftVelocity = leftFlywheelMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.leftClosedLoopError = leftFlywheelMotor.getClosedLoopError(SLOT_INDEX);
    inputs.leftAppliedVolts = leftFlywheelMotor.getMotorOutputVoltage();
    inputs.leftCurrentAmps = new double[] {leftFlywheelMotor.getStatorCurrent()};
    inputs.leftTempCelcius = new double[] {leftFlywheelMotor.getTemperature()};

    inputs.rightVelocity = rightFlywheelMotor.getSelectedSensorVelocity(SLOT_INDEX);
    inputs.rightClosedLoopError = rightFlywheelMotor.getClosedLoopError(SLOT_INDEX);
    inputs.rightAppliedVolts = rightFlywheelMotor.getMotorOutputVoltage();
    inputs.rightCurrentAmps = new double[] {rightFlywheelMotor.getStatorCurrent()};
    inputs.rightTempCelcius = new double[] {rightFlywheelMotor.getTemperature()};
  }

  @Override
  public void setMotorPercentage(double percentage) {
    leftFlywheelMotor.set(ControlMode.PercentOutput, percentage);
    rightFlywheelMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setVelocity(double velocity) {
    leftFlywheelMotor.follow(this.rightFlywheelMotor);
    rightFlywheelMotor.set(TalonFXControlMode.Velocity, velocity);
  }

  @Override
  public void configureKF(double kF) {
    rightFlywheelMotor.config_kF(SLOT_INDEX, kF, TIMEOUT_MS);
  }

  @Override
  public void configureKP(double kP) {
    rightFlywheelMotor.config_kP(SLOT_INDEX, kP, TIMEOUT_MS);
  }

  @Override
  public void configureKI(double kI) {
    rightFlywheelMotor.config_kI(SLOT_INDEX, kI, TIMEOUT_MS);
  }

  @Override
  public void configureKD(double kD) {
    rightFlywheelMotor.config_kD(SLOT_INDEX, kD, TIMEOUT_MS);
  }
}
