// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

public class ElevatorIOTalonFX implements ElevatorIO {
  private boolean isControlEnabled;

  private final WPI_TalonFX leftElevatorMotor;
  private final WPI_TalonFX rightElevatorMotor;
  private final Pigeon2 pigeon;

  public ElevatorIOTalonFX() {

    this.isControlEnabled = false;

    this.leftElevatorMotor = new WPI_TalonFX(LEFT_ELEVATOR_MOTOR_CAN_ID);
    this.rightElevatorMotor = new WPI_TalonFX(RIGHT_ELEVATOR_MOTOR_CAN_ID);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behaviour */
    this.rightElevatorMotor.configFactoryDefault();
    this.leftElevatorMotor.configFactoryDefault();

    /** Config Objects for motor controllers */
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    /* Disable all motors */
    this.rightElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
    this.leftElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);

    /* Set neutral modes */
    this.leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    this.rightElevatorMotor.setNeutralMode(NeutralMode.Brake);

    this.leftElevatorMotor.follow(this.rightElevatorMotor);

    /* Configure output */
    this.rightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);
    this.leftElevatorMotor.setInverted(TalonFXInvertType.FollowMaster);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     *
     * this.leftElevatorMotor.setSensorPhase(true)
     * this.rightElevatorMotor.setSensorPhase(true)
     */

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    rightConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    /* FPID for Distance */
    rightConfig.slot0.kF = POSITION_PID_F;
    rightConfig.slot0.kP = POSITION_PID_P;
    rightConfig.slot0.kI = POSITION_PID_I;
    rightConfig.slot0.kD = POSITION_PID_D;
    rightConfig.slot0.integralZone = POSITION_PID_I_ZONE;
    rightConfig.slot0.closedLoopPeakOutput = POSITION_PID_PEAK_OUTPUT;

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

    /* Motion Magic Configs */
    rightConfig.motionAcceleration =
        ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    rightConfig.motionCruiseVelocity = MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    rightConfig.motionCurveStrength = SCURVE_STRENGTH;

    /* APPLY the config settings */
    this.rightElevatorMotor.configAllSettings(rightConfig);

    /* Initialize */
    this.rightElevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.leftElevatorMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.leftElevatorMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.pigeon = new Pigeon2(PIGEON_ID);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.isControlEnabled = isControlEnabled;

    inputs.leftPosition = leftElevatorMotor.getSelectedSensorPosition();
    inputs.leftVelocity = leftElevatorMotor.getSelectedSensorVelocity();
    inputs.leftClosedLoopError = leftElevatorMotor.getClosedLoopError();
    inputs.leftAppliedVolts = leftElevatorMotor.getMotorOutputVoltage();
    inputs.leftCurrentAmps = new double[] {leftElevatorMotor.getStatorCurrent()};
    inputs.leftTempCelcius = new double[] {leftElevatorMotor.getTemperature()};

    inputs.rightPosition = rightElevatorMotor.getSelectedSensorPosition();
    inputs.rightVelocity = rightElevatorMotor.getSelectedSensorVelocity();
    inputs.rightClosedLoopError = rightElevatorMotor.getClosedLoopError();
    inputs.rightAppliedVolts = rightElevatorMotor.getMotorOutputVoltage();
    inputs.rightCurrentAmps = new double[] {rightElevatorMotor.getStatorCurrent()};
    inputs.rightTempCelcius = new double[] {rightElevatorMotor.getTemperature()};

    inputs.pitch = pigeon.getPitch();
  }

  @Override
  public void setControlEnabled(boolean controlEnabled) {
    isControlEnabled = controlEnabled;
  }

  @Override
  public void setMotorPercentage(double percentage) {
    leftElevatorMotor.set(ControlMode.PercentOutput, percentage);
    rightElevatorMotor.set(ControlMode.PercentOutput, percentage);
  }

  @Override
  public void setPosition(double position, double arbitraryFeedForward) {
    leftElevatorMotor.follow(rightElevatorMotor);
    rightElevatorMotor.set(
        TalonFXControlMode.Position,
        position,
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  @Override
  public void configureKF(double kF) {
    rightElevatorMotor.config_kF(SLOT_INDEX, kF, TIMEOUT_MS);
  }

  @Override
  public void configureKP(double kP) {
    rightElevatorMotor.config_kP(SLOT_INDEX, kP, TIMEOUT_MS);
  }

  @Override
  public void configureKI(double kI) {
    rightElevatorMotor.config_kI(SLOT_INDEX, kI, TIMEOUT_MS);
  }

  @Override
  public void configureKD(double kD) {
    rightElevatorMotor.config_kD(SLOT_INDEX, kD, TIMEOUT_MS);
  }

  @Override
  public void configClosedLoopPeakOutput(double peakOutput) {
    rightElevatorMotor.configClosedLoopPeakOutput(SLOT_INDEX, peakOutput);
  }
}
