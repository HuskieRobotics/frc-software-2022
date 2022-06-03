package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;

/**
 * This command, when executed, instructs the drivetrain subsystem to move based on the joystick
 * inputs. If the hub is visible, the drivetrain will rotate to stay aimed at the hub; if not, the
 * joystick input will control the rotation of the drivetrain.
 *
 * <p>Requires: the drivetrain and flywheel subsystems (the collector subsystem is not a requirement
 * as it is not controlled and only queried for its current state)
 *
 * <p>Finished When: the drivetrain is aimed, the flywheel is at the specified speed, and the robot
 * is within optimal shooting distance
 *
 * <p>At End: stops the drivetrain
 */
public class LimelightAlignOnMoveCommand extends CommandBase {

  private PIDController controller;
  private DrivetrainSubsystem drivetrainSubsystem;
  private Flywheel flywheelSubsystem;
  private Collector collectorSubsystem;
  private Joystick joystick0;
  private Joystick joystick1;

  // update constructor to use suppliers instead of Joysticks references like the default drive
  // command
  /**
   * Constructs a new LimelightAlignOnMoveCommand object.
   *
   * @param drivetrain the drivetrain subsystem this command will control
   * @param flywheel the flywheel subsystem this command will control
   * @param collector the collector subsystem to query for its current state (not a requirement)
   * @param joystick0 the joystick that controls the drivetrain's x and y translation
   * @param joystick1 the joystick that controls the drivetrain's rotation
   */
  public LimelightAlignOnMoveCommand(
      DrivetrainSubsystem drivetrain,
      Flywheel flywheel,
      Collector collector,
      Joystick joystick0,
      Joystick joystick1) {
    controller =
        new PIDController(DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0);
    drivetrainSubsystem = drivetrain;
    flywheelSubsystem = flywheel;
    collectorSubsystem = collector;
    this.joystick0 = joystick0;
    this.joystick1 = joystick1;

    addRequirements(drivetrainSubsystem);
    addRequirements(flywheelSubsystem);
    // don't add the storage as a requirement as we are only determining if cargo is indexed
  }

  /**
   * This method is invoked once when this command is scheduled. It resets the PID controller for
   * the drivetrain rotation. It is critical that this initialization occurs in this method and not
   * the constructor as this command is constructed once when the RobotContainer is created, but
   * this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    // critical to reset the PID controller each time this command is initialized to reset any
    //  accumulated values due to non-zero I or D values
    controller.reset();

    // delete this method
    drivetrainSubsystem.enableAutoAimAndShoot();
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly instructs
   * the drivetrain subsytem to move translationally while keeping aligned to the hub, if visible.
   * If the hub is not visible, the joystick input controls the rotation of the drivetrain.
   */
  @Override
  public void execute() {
    // the input to the rotational PID controller is the number of degrees between the rotation
    //  of the drivetrain and the center of the hub target
    double output = controller.calculate(drivetrainSubsystem.getLimelightX(), 0);

    // this constant can be used to decrease the maximum translational speed of the drivetrain
    final int AIMING_LINEAR_SPEED_MULTIPLIER = 1;

    // if the target is visible, try to aim with PID
    if (drivetrainSubsystem.isLimelightTargetVisible()) {

      // refer to this document for a detailed explanation of this algorithm:
      //  https://docs.google.com/document/d/1WtUOrvnbNTLbmrZ3Far-ipM_e6wJMzDyVqbghETleNs/edit

      // limelight offset angle in rad, left of camera is negative angle
      double tx = Math.toRadians(drivetrainSubsystem.getLimelightX());

      // positive limelight distance in meters; add the distance from the limelight to the center of
      // the robot
      //  and the distance for the retroreflective tape to the center of the hub before converting
      // to meters
      double d =
          (drivetrainSubsystem.getLimelightDistanceIn()
                  + LimelightConstants.EDGE_TO_CENTER_HUB_DISTANCE)
              * 0.0254;

      // assuming robot v is forward/left positive and in the direction of the collector
      double dhdt = drivetrainSubsystem.getVelocityX();
      double dldt = drivetrainSubsystem.getVelocityY();
      double h = d * Math.cos(tx); // h is always positive
      double l = -d * Math.sin(tx); // positive if the target is to the left of the robot

      double w = -(l * dhdt - h * dldt) / (d * d); // negative dtheta/dt, assuming positive ccw

      drivetrainSubsystem.aim(
          -RobotContainer.modifyAxis(joystick0.getY())
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
              * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick x
          -RobotContainer.modifyAxis(joystick0.getX())
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
              * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick y
          output + w // PID output (rad/s)
          );

      // only turn on the flywheel if there is a cargo indexed
      if (!collectorSubsystem.isEnabled()) {
        flywheelSubsystem.setVelocity(drivetrainSubsystem.getVelocityFromLimelight());
      }

    }
    // if we can't see the target, the joystick controls the rotation
    else {
      // if we don't see the target, reset the PID controller for any accumulated values due
      //  to non-zero I or D values
      controller.reset();

      drivetrainSubsystem.drive(
          -RobotContainer.modifyAxis(joystick0.getY())
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, // joystick x
          -RobotContainer.modifyAxis(joystick0.getX())
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, // joystick y
          -RobotContainer.modifyAxis(joystick1.getX())
              * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // joystick rotation
    }
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain. It does not stop the motion of the flywheel.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
    drivetrainSubsystem.disableAutoAimAndShoot();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * drivetrain is aimed, the flywheel is at the specified speed, and the robot is within optimal
   * shooting distance.
   */
  @Override
  public boolean isFinished() {
    // shooter is aimed AND flywheel is at speed AND robot is within optimal shooting distance
    return drivetrainSubsystem.isLimelightTargetVisible()
        && drivetrainSubsystem.isAimed()
        && flywheelSubsystem.isAtSetpoint()
        && (drivetrainSubsystem.getLimelightDistanceIn()
                < LimelightConstants.AUTO_SHOT_HUB_FAR_DISTANCE
            && drivetrainSubsystem.getLimelightDistanceIn()
                > LimelightConstants.AUTO_SHOT_HUB_CLOSE_DISTANCE);
  }
}
