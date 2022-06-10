// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AutoConstants;

public class DrivetrainIOTalonFX implements DrivetrainIO {
  private final Pigeon2 pigeon;
  private double gyroOffset;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private SimpleMotorFeedforward feedForward;

  public DrivetrainIOTalonFX() {
    this.pigeon = new Pigeon2(PIGEON_ID);
    this.gyroOffset = 0.0;

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    // Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
    // and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
    // Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
    // class.

    // By default we will use Falcon 500s in standard configuration. But if you use
    // a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.

    frontLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    frontRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);

    backLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);

    backRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    this.feedForward =
        new SimpleMotorFeedforward(
            AutoConstants.S_VOLTS,
            AutoConstants.V_VOLT_SECONDS_PER_METER,
            AutoConstants.A_VOLT_SECONDS_SQUARED_PER_METER);
  }

  @Override
  public void updateInputs(DrivetrainIOInputs inputs) {

    inputs.frontLeftDriveVelocity = frontLeftModule.getDriveVelocity();
    inputs.frontLeftSteerAngle = frontLeftModule.getSteerAngle();
    inputs.frontRightDriveVelocity = frontRightModule.getDriveVelocity();
    inputs.frontRightSteerAngle = frontRightModule.getSteerAngle();
    inputs.backLeftDriveVelocity = backLeftModule.getDriveVelocity();
    inputs.backLeftSteerAngle = backLeftModule.getSteerAngle();
    inputs.backRightDriveVelocity = backRightModule.getDriveVelocity();
    inputs.backRightSteerAngle = backRightModule.getSteerAngle();
    inputs.yaw = pigeon.getYaw() + this.gyroOffset;
    inputs.limelightTX =
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NETWORK_TABLE_NAME)
            .getEntry("tx")
            .getDouble(0);
    inputs.limelightTY =
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NETWORK_TABLE_NAME)
            .getEntry("ty")
            .getDouble(0);
    inputs.limelightTV =
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NETWORK_TABLE_NAME)
            .getEntry("tv")
            .getDouble(0);
  }

  @Override
  public void setGyroOffset(double expectedYaw) {
    gyroOffset = expectedYaw - pigeon.getYaw();
  }

  @Override
  public void setSwerveModules(SwerveModuleState[] states) {

    // the set method of the swerve modules take a voltage, not a velocity
    frontLeftModule.set(
        states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());
    frontRightModule.set(
        states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    backLeftModule.set(
        states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());
    backRightModule.set(
        states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());
  }

  @Override
  public void setSwerveModulesWithFeedForward(SwerveModuleState[] states) {
    // the set method of the swerve modules take a voltage, not a velocity
    frontLeftModule.set(
        this.calculateFeedforwardVoltage(states[0].speedMetersPerSecond),
        states[0].angle.getRadians());
    frontRightModule.set(
        this.calculateFeedforwardVoltage(states[1].speedMetersPerSecond),
        states[1].angle.getRadians());
    backLeftModule.set(
        this.calculateFeedforwardVoltage(states[2].speedMetersPerSecond),
        states[2].angle.getRadians());
    backRightModule.set(
        this.calculateFeedforwardVoltage(states[3].speedMetersPerSecond),
        states[3].angle.getRadians());
  }

  private double calculateFeedforwardVoltage(double velocity) {
    double voltage = this.feedForward.calculate(velocity);
    // clamp the voltage to the maximum voltage
    if (voltage > MAX_VOLTAGE) {
      return MAX_VOLTAGE;
    }
    return voltage;
  }
}
