package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

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

  private final PIDController controller;
  private final Drivetrain drivetrain;
  private final Flywheel flywheel;
  private final Collector collector;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  /**
   * Constructs a new LimelightAlignOnMoveCommand object.
   *
   * @param drivetrain the drivetrain subsystem this command will control
   * @param flywheel the flywheel subsystem this command will control
   * @param collector the collector subsystem to query for its current state (not a requirement)
   * @param translationXSupplier supplies the desired velocity in the x direction (m/s)
   * @param translationYSupplier supplies the desired velocity in the y direction (m/s)
   * @param rotationSupplier supplies the desried rotational velocity (m/s)
   */
  public LimelightAlignOnMoveCommand(
      Drivetrain drivetrain,
      Flywheel flywheel,
      Collector collector,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.controller =
        new PIDController(
            DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0, LOOP_PERIOD_SECS);
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.collector = collector;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
    addRequirements(flywheel);
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
    drivetrain.enableAutoAimAndShoot();
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
    double output = controller.calculate(drivetrain.getLimelightX(), 0);

    // if the target is visible, try to aim with PID
    if (drivetrain.isLimelightTargetVisible()) {

      // refer to this document for a detailed explanation of this algorithm:
      //  https://docs.google.com/document/d/1WtUOrvnbNTLbmrZ3Far-ipM_e6wJMzDyVqbghETleNs/edit

      // limelight offset angle in rad, left of camera is negative angle
      double tx = Math.toRadians(drivetrain.getLimelightX());

      // positive limelight distance in meters; add the distance from the limelight to the center of
      // the robot
      //  and the distance for the retroreflective tape to the center of the hub before converting
      // to meters
      double d =
          (drivetrain.getLimelightDistanceIn() + LimelightConstants.EDGE_TO_CENTER_HUB_DISTANCE)
              * 0.0254;

      // assuming robot v is forward/left positive and in the direction of the collector
      double dhdt = drivetrain.getVelocityX();
      double dldt = drivetrain.getVelocityY();
      double h = d * Math.cos(tx); // h is always positive
      double l = -d * Math.sin(tx); // positive if the target is to the left of the robot

      double w = -(l * dhdt - h * dldt) / (d * d); // negative dtheta/dt, assuming positive ccw

      drivetrain.aim(
          translationXSupplier.getAsDouble(),
          translationYSupplier.getAsDouble(),
          output + w // PID output (rad/s)
          );

      // only turn on the flywheel if there is a cargo indexed
      if (!collector.isEnabled()) {
        flywheel.setVelocity(drivetrain.getVelocityFromLimelight());
      }

    }
    // if we can't see the target, the joystick controls the rotation
    else {
      // if we don't see the target, reset the PID controller for any accumulated values due
      //  to non-zero I or D values
      controller.reset();

      drivetrain.drive(
          translationXSupplier.getAsDouble(),
          translationYSupplier.getAsDouble(),
          rotationSupplier.getAsDouble());
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
    drivetrain.stop();
    drivetrain.disableAutoAimAndShoot();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * drivetrain is aimed, the flywheel is at the specified speed, and the robot is within optimal
   * shooting distance.
   */
  @Override
  public boolean isFinished() {
    // shooter is aimed AND flywheel is at speed AND robot is within optimal shooting distance
    return drivetrain.isLimelightTargetVisible()
        && drivetrain.isAimed()
        && flywheel.isAtSetpoint()
        && (drivetrain.getLimelightDistanceIn() < LimelightConstants.AUTO_SHOT_HUB_FAR_DISTANCE
            && drivetrain.getLimelightDistanceIn()
                > LimelightConstants.AUTO_SHOT_HUB_CLOSE_DISTANCE);
  }
}
