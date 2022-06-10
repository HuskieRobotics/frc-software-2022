// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.commands.LimelightAlignToTargetCommand;
import frc.robot.commands.LimelightAlignWithGyroCommand;
import frc.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four MK4 swerve modules,
 * each with two motors and an encoder. It also consists of a Pigeon which is used to measure the
 * robot's rotation.
 */
public class Drivetrain extends SubsystemBase {
  private final DrivetrainIO io;
  private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  // some of this code is from the SDS example code

  // When aiming, rotate the robot much slower to avoid overshooting the setpoint
  public static final double MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 1.0;

  private Translation2d centerGravity;

  /* The geometry and coordinate systems can be confusing. Refer to this document
  for a detailed explanation: !!!
  */
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));

  private boolean isFieldRelative;
  private boolean isXstance;

  private final SwerveDriveOdometry odometry;

  private ChassisSpeeds chassisSpeeds;

  private int aimSetpointCount;
  private int gyroAimSetpointCount;
  private double lastLimelightDistance;
  private boolean limelightAimEnabled;
  private boolean autoAimAndShootEnabled;
  private double gyroSetpoint;

  private static final String SUBSYSTEM_NAME = "Drivetrain";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new DrivetrainSubsystem object. */
  public Drivetrain(DrivetrainIO io) {
    this.io = io;

    this.centerGravity = new Translation2d(); // default to (0,0)

    this.zeroGyroscope();

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");

    this.isFieldRelative = false;
    this.isXstance = false;

    this.odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(inputs.yaw));

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.aimSetpointCount = 0;
    this.gyroAimSetpointCount = 0;
    this.lastLimelightDistance = 0.0;
    this.limelightAimEnabled = true;
    this.autoAimAndShootEnabled = false;
    this.gyroSetpoint = 0.0;

    tabMain.addNumber("Limelight Dist", () -> getLimelightDistanceIn());
    tabMain.addBoolean("Target Visible?", () -> isLimelightTargetVisible());
    tabMain.addNumber("Limelight Vel", () -> getVelocityFromLimelight());
    tabMain.addBoolean("At Launchpad Dist?", () -> isAtLaunchpadDistance());
    tabMain.addBoolean("At Wall Dist?", () -> isAtWallDistance());
    tabMain.addBoolean("Is Aimed?", () -> isAimed());
    tabMain.addBoolean("Is Aimed Gryo?", () -> isAimedWithGyro());
    tabMain.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    tabMain.addBoolean("X-Stance On?", this::isXstance);
    tabMain.addBoolean("Limelight Aim Enabled?", this::isLimelightAimEnabled);
    tabMain.addBoolean("Aim & Shoot Enabled?", () -> this.autoAimAndShootEnabled);
    tabMain.addBoolean("Field-Relative Enabled?", () -> this.isFieldRelative);

    if (DEBUGGING) {
      tab.add(SUBSYSTEM_NAME, this);
      tab.addNumber("vx", () -> getVelocityX());
      tab.addNumber("vy", () -> getVelocityY());
      tab.addNumber("Limelight x", () -> getLimelightX());
      tab.addNumber("Limelight y", () -> getLimelighty());
      tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
      tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
      tab.addNumber("Pose Rotation", () -> odometry.getPoseMeters().getRotation().getDegrees());
      tab.addNumber("CoG X", () -> this.centerGravity.getX());
      tab.addNumber("CoG Y", () -> this.centerGravity.getY());
      tab.addNumber("gyro setpoint", () -> this.gyroSetpoint);
    }

    if (TESTING) {
      tab.add("Enable XStance", new InstantCommand(() -> this.enableXstance()));
      tab.add("Disable XStance", new InstantCommand(() -> this.disableXstance()));
      tab.add("Align with Gyro", new LimelightAlignWithGyroCommand(this));
      tab.add("Align to Target", new LimelightAlignToTargetCommand(this));
    }
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    // There is a delay between setting the yaw on the Pigeon and that change
    //      taking effect. As a result, it is recommended to never set the yaw and
    //      adjust the local offset instead.
    setGyroOffset(0.0);
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. This method should always be invoked instead of obtaining the yaw directly from the
   * Pigeon as the local offset needs to be added.
   *
   * @return the rotation of the robot
   */
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(inputs.yaw);
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(double expectedYaw) {
    // There is a delay between setting the yaw on the Pigeon and that change
    //      taking effect. As a result, it is recommended to never set the yaw and
    //      adjust the local offset instead.
    io.setGyroOffset(expectedYaw);
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
   * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
  public void resetOdometry(PathPlannerState state) {
    setGyroOffset(state.holonomicRotation.getDegrees());
    odometry.resetPosition(
        new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation),
        this.getGyroscopeRotation());
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference of the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
   * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * @param translationXSupplier the desired velocity in the x direction (m/s)
   * @param translationYSupplier the desired velocity in the y direction (m/s)
   * @param rotationSupplier the desired rotational velcoity (rad/s)
   */
  public void drive(
      double translationXSupplier, double translationYSupplier, double rotationSupplier) {
    if (isXstance) {
      this.setXStance();
    } else {
      if (isFieldRelative) {
        chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier,
                translationYSupplier,
                rotationSupplier,
                getGyroscopeRotation());

      } else {
        chassisSpeeds =
            new ChassisSpeeds(translationXSupplier, translationYSupplier, rotationSupplier);
      }

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);

      io.setSwerveModules(states);
    }
  }

  /**
   * Stops the motion of the robot. Since the motors are in break mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    setSwerveModuleStates(states);
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the drivetrain needs to
   * continually update the odometry of the robot.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);

    odometry.update(
        this.getGyroscopeRotation(),
        new SwerveModuleState(
            inputs.frontLeftDriveVelocity, new Rotation2d(inputs.frontLeftSteerAngle)),
        new SwerveModuleState(
            inputs.frontRightDriveVelocity, new Rotation2d(inputs.frontRightSteerAngle)),
        new SwerveModuleState(
            inputs.backLeftDriveVelocity, new Rotation2d(inputs.backLeftSteerAngle)),
        new SwerveModuleState(
            inputs.backRightDriveVelocity, new Rotation2d(inputs.backRightSteerAngle)));

    // Log poses
    Pose2d pose = odometry.getPoseMeters();
    Logger.getInstance()
        .recordOutput(
            "Odometry/Robot",
            new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    Logger.getInstance()
        .recordOutput(
            "Odometry/VisionTarget",
            new double[] {FieldConstants.HUB_CENTER.getX(), FieldConstants.HUB_CENTER.getY()});
    if (isLimelightTargetVisible()) {
      Logger.getInstance().recordOutput("Vision/DistanceInches", getLimelightDistanceIn());
    }
  }

  /**
   * Sets each of the swerve modules based on the specified corresponding swerve module state.
   * Incorporates the configured feedforward when setting each swerve module. The order of the
   * states in the array must be front left, front right, back left, back right.
   *
   * @param states the specified swerve module state for each swerve module
   */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    io.setSwerveModulesWithFeedForward(states);
  }

  /**
   * Returns the kinematics for the drivetrain
   *
   * @return the kinematics for the drivetrain
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns true if field relative mode is enabled
   *
   * @return true if field relative mode is enabled
   */
  public boolean getFieldRelative() {
    return isFieldRelative;
  }

  /**
   * Enables field-relative mode. When enabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the field.
   */
  public void enableFieldRelative() {
    this.isFieldRelative = true;
  }

  /**
   * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the robot.
   */
  public void disableFieldRelative() {
    this.isFieldRelative = false;
  }

  /**
   * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting.
   */
  public void setXStance() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
    states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
    states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
    states[3].angle =
        new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(TRACKWIDTH_METERS / WHEELBASE_METERS));
    io.setSwerveModules(states);
  }

  /**
   * Sets the robot's center of gravity about which it will rotate. The origin is at the center of
   * the robot. The positive x direction is forward; the positive y direction, left.
   *
   * @param x the x coordinate of the robot's center of gravity (in meters)
   * @param y the y coordinate of the robot's center of gravity (in meters)
   */
  public void setCenterGrav(double x, double y) {
    this.centerGravity = new Translation2d(x, y);
  }

  /** Resets the robot's center of gravity about which it will rotate to the center of the robot. */
  public void resetCenterGrav() {
    setCenterGrav(0.0, 0.0);
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. Instead of rotating about the robot's center of gravity, the robot will
   * rotate about the point on the robot's frame perimeter in the direction of the velocity. This
   * will result in a "spin move" to evade a defending robot. The velocities must be specified from
   * reference of the field's frame of reference: the origin of the field to the lower left corner
   * (i.e., the corner of the field to the driver's right). Zero degrees is away from the driver and
   * increases in the CCW direction.
   *
   * <p>This method needs to be debugged.
   *
   * @param translationX the desired velocity in the x direction (m/s)
   * @param translationY the desired velocity in the y direction (m/s)
   * @param rotation the desired rotational velcoity (rad/s)
   */
  public void rotateEvasively(double translationX, double translationY, double rotation) {

    double gyro = getGyroscopeRotation().getDegrees();

    /*
            Assumptions:
                    * positive x is moving joystick 0 right (no it's positive when left)
                    * positive y is moving joystick 0 forward (correct)
                    * positive z is moving joystick 1 left (correct)
                    * gyro values increase when rotation counter clockwise
    */

    double x = -translationX;
    double y = translationY;
    double z = rotation;

    double worldFrameAngle = Math.toDegrees(Math.atan(y / x));
    if (x < 0) {
      worldFrameAngle += 180.0;
    }
    double robotFrameAngle = worldFrameAngle - gyro;
    double robotFrameCOGAngle;

    if (z < 0) {
      robotFrameCOGAngle = robotFrameAngle + COG_OFFSET;
    } else {
      robotFrameCOGAngle = robotFrameAngle - COG_OFFSET;
    }

    double cogX =
        Math.cos(Math.toRadians(robotFrameCOGAngle)) * EVASIVE_ROTATION_COG_SHIFT_MAGNITUDE;
    double cogY =
        Math.sin(Math.toRadians(robotFrameCOGAngle)) * EVASIVE_ROTATION_COG_SHIFT_MAGNITUDE;

    setCenterGrav(cogX, cogY);
  }

  /**
   * Returns the horizontal offset between the rotation of the robot and the center of the hub's
   * vision target (in degrees). A positive value indicates the hub's vision target is to the left
   * of the robot rotation.
   *
   * @return the horizontal offset between the rotation of the robot and the center of the hub's
   *     vision target (in degrees)
   */
  public double getLimelightX() {
    return inputs.limelightTX;
  }

  /**
   * Returns the vertical offset between the center of the Limelight image and the the hub's vision
   * target (in degrees). A positive value indicates the vision target is above the center of the
   * Limelight image.
   *
   * @return the vertical offset between the center of the Limelight image and the the hub's vision
   *     target (in degrees)
   */
  private double getLimelighty() {
    return inputs.limelightTY;
  }

  /**
   * Returns true if the hub's vision target is visible.
   *
   * @return true if the hub's vision target is visible
   */
  public boolean isLimelightTargetVisible() {
    return inputs.limelightTV == 1;
  }

  /**
   * Returns the distance from the Limelight mounted on the robot to the hub's vision target (in
   * inches).
   *
   * @return the distance from the Limelight mounted on the robot to the hub's vision target (in
   *     inches).
   */
  public double getLimelightDistanceIn() {
    // refer to the following page for the algorithm to calculate the distance:
    //      https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

    double ty = inputs.limelightTY;

    this.lastLimelightDistance =
        (LimelightConstants.HUB_H - LimelightConstants.ROBOT_H)
            / (Math.tan(
                Math.toRadians(
                    LimelightConstants.LIMELIGHT_MOUNT_ANGLE
                        + LimelightConstants.LIMELIGHT_ANGLE_OFFSET
                        + ty)));

    return this.lastLimelightDistance;
  }

  /**
   * Returns the desired flywheel velocity (in ticks / 100 ms) based on the distance to the hub. The
   * line of best fit was determined empirically.
   *
   * @return the desired flywheel velocity (in ticks / 100 ms) based on the distance to the hub
   */
  public double getVelocityFromLimelight() {
    return LIMELIGHT_SLOPE * this.getLimelightDistanceIn() + LIMELIGHT_Y_COMPONENT;
  }

  private boolean isAtLaunchpadDistance() {
    return Math.abs(LimelightConstants.HUB_LAUNCHPAD_DISTANCE - this.lastLimelightDistance)
        <= LimelightConstants.DISTANCE_TOLERANCE;
  }

  private boolean isAtWallDistance() {
    return Math.abs(LimelightConstants.HUB_WALL_DISTANCE - this.lastLimelightDistance)
        <= LimelightConstants.DISTANCE_TOLERANCE;
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. This method is designed to be invoked from commands that use joystick
   * inputs for the x and y velocities and a PID for the rotational velocity (to align the robot
   * with the hub).
   *
   * <p>The velocities may be specified from either the robot's frame of reference of the field's
   * frame of reference. In the robot's frame of reference, the positive x direction is forward; the
   * positive y direction, left; position rotation, CCW. In the field frame of reference, the origin
   * of the field to the lower left corner (i.e., the corner of the field to the driver's right).
   * Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param translationXSupplier the desired velocity in the x direction (m/s)
   * @param translationYSupplier the desired velocity in the y direction (m/s)
   * @param rotationSupplier the desired rotational velcoity (rad/s)
   */
  public void aim(
      double translationXSupplier, double translationYSupplier, double rotationSupplier) {

    // incorporate a feedforward for the rotation to make the PID more responsive

    // LIMELIGHT_F is specified as a fraction of MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    if (rotationSupplier > 0) { // clockwise
      rotationSupplier += LIMELIGHT_F * MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else if (rotationSupplier < 0) { // counterclockwise
      rotationSupplier -= LIMELIGHT_F * MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // clamp the rotation to MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    if (rotationSupplier > MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
      rotationSupplier = MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else if (rotationSupplier < -MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND) {
      rotationSupplier = -MAX_AIM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    drive(translationXSupplier, translationYSupplier, rotationSupplier);
  }

  /**
   * Returns true if the robot is aimed at the center of the hub's vision traget (within the desired
   * tolerance) based on the Limelight's reported horizontal offset. If Limelight aiming is
   * disabled, this method returns true.
   *
   * @return true if the robot is aimed at the center of the hub's vision traget (within the desired
   *     tolerance) based on the Limelight's reported horizontal offset. If Limelight aiming is
   *     disabled, this method returns true
   */
  public boolean isAimed() {

    // check if limelight aiming is enabled
    if (!this.limelightAimEnabled) {
      return true;
    }

    // Calculate the horizontal offset (in inches) from the center of the hub and
    //      ensure we are within the specified tolerance. It is critical to use the
    //      offset in inches instead of degrees as the same degree offset when close
    //      to the hub or far from the hub results in significantly different offsets
    //      in inches, which can result in missed shots. The tolerance is critical
    //      since it is highly unlikely that the robot will be aligned perfectly.
    //      Waiting the specified number of iterations is critical since the PID may
    //      overshoot the setpoint and need additional time to settle. The robot is
    //      only considered aimed if it remains aligned continuously for the desired
    //      number of iterations. Without waiting, it would be reported that the
    //      robot was aimed but then, when the cargo is shot, the robot would
    //      overrotate and miss.

    double distanceToHub =
        getLimelightDistanceIn() + LimelightConstants.EDGE_TO_CENTER_HUB_DISTANCE;
    if (Math.abs(distanceToHub * Math.sin(Math.toRadians(getLimelightX())))
        < LIMELIGHT_AIM_TOLERANCE) {
      aimSetpointCount++;
      if (aimSetpointCount >= AIM_SETPOINT_COUNT) {
        return true;
      }
    } else {

      aimSetpointCount = 0;
    }
    return false;
  }

  /**
   * Sets the setpoint for the robot's rotation to the specified value (in degrees). Zero degrees is
   * away from the driver and increases in the CCW direction.
   *
   * @param setpoint the specified setpoint for the robot's rotation (in degrees)
   */
  public void setGyroSetpoint(double setpoint) {
    this.gyroSetpoint = setpoint;
  }

  /**
   * Returns true if the robot is aimed at the center of the hub's vision traget (within the desired
   * tolerance) based on the gyro. If Limelight aiming is disabled, this method returns true.
   *
   * @return true if the robot is aimed at the center of the hub's vision traget (within the desired
   *     tolerance) based on the gyro. If Limelight aiming is disabled, this method returns true.
   */
  public boolean isAimedWithGyro() {

    // check if limelight aiming is enabled
    if (!this.limelightAimEnabled) {
      return true;
    }

    // Calculate the horizontal offset (in inches) from the center of the hub and
    //      ensure we are within the specified tolerance. It is critical to use the
    //      offset in inches instead of degrees as the same degree offset when close
    //      to the hub or far from the hub results in significantly different offsets
    //      in inches, which can result in missed shots. The tolerance is critical
    //      since it is highly unlikely that the robot will be aligned perfectly.
    //      Waiting the specified number of iterations is critical since the PID may
    //      overshoot the setpoint and need additional time to settle. The robot is
    //      only considered aimed if it remains aligned continuously for the desired
    //      number of iterations. Without waiting, it would be reported that the
    //      robot was aimed but then, when the cargo is shot, the robot would
    //      overrotate and miss.

    double distanceToHub =
        getLimelightDistanceIn() + LimelightConstants.EDGE_TO_CENTER_HUB_DISTANCE;
    double setPointDisplacement = distanceToHub * Math.sin(Math.toRadians(this.gyroSetpoint));
    double currentDisplacement = distanceToHub * Math.sin(getGyroscopeRotation().getRadians());
    if (Math.abs(setPointDisplacement - currentDisplacement) < LIMELIGHT_AIM_TOLERANCE) {
      gyroAimSetpointCount++;
      if (gyroAimSetpointCount >= AIM_SETPOINT_COUNT) {
        return true;
      }
    } else {

      gyroAimSetpointCount = 0;
    }
    return false;
  }

  /**
   * Returns the desired velocity of the drivetrain in the x direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the x direction (units of m/s)
   */
  public double getVelocityX() {
    return chassisSpeeds.vxMetersPerSecond;
  }

  /**
   * Returns the desired velocity of the drivetrain in the y direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the y direction (units of m/s)
   */
  public double getVelocityY() {
    return chassisSpeeds.vyMetersPerSecond;
  }

  /**
   * Enable the auto aim and shoot mode. In this mode, the robot moves based on the joystick inputs.
   * If the hub is visible, the drivetrain will rotate to stay aimed at the hub; if not, the
   * joystick input will control the rotation of the drivetrain.
   */
  public void enableAutoAimAndShoot() {
    this.autoAimAndShootEnabled = true;
  }

  /** Disables the auto aim and shoot mode. */
  public void disableAutoAimAndShoot() {
    this.autoAimAndShootEnabled = false;
  }

  /** Enables aiming based on the Limelight. */
  public void enableLimelightAim() {
    this.limelightAimEnabled = true;
  }

  /**
   * Disables aiming based on the Limelight. This method is only invoked when the operator has
   * determined that the Limelight is not functioning and the driver will aim the robot manually.
   * This is critical since, without disabling this feature, the driver would be fighting the code
   * that tries to aim based on the malfunctioning Limelight.
   */
  public void disableLimelightAim() {
    this.limelightAimEnabled = false;
  }

  /**
   * Returns true if the limelight aim feature is enabled.
   *
   * @return
   */
  public boolean isLimelightAimEnabled() {
    return this.limelightAimEnabled;
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting. The robot cannot be driven until x-stance is disabled.
   */
  public void enableXstance() {
    this.isXstance = true;
    this.setXStance();
  }

  /** Disables the x-stance, allowing the robot to be driven. */
  public void disableXstance() {
    this.isXstance = false;
  }

  /**
   * Returns true if the robot is in the x-stance orientation.
   *
   * @return true if the robot is in the x-stance orientation
   */
  public boolean isXstance() {
    return isXstance;
  }
}
