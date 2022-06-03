package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;

/**
 * This command, when executed, instructs the flywheel subsystem to spin the flywheel at a velocity
 * based on the distance from the robot to the hub as determined by the Limelight. It does not stop
 * the flywheel when it ends as it is intended to be used as part of sequence of commands and will
 * be followed by a command to shoot the cargo.
 *
 * <p>Requires: the flywheel subsystem (the drivetrain subsystem is not required as it is only
 * queried to get the distance to the hub)
 *
 * <p>Finished When: the flywheel velocity is at the setpoint
 *
 * <p>At End: leaves the flywheel spinning
 */
public class LimelightSetFlywheelVelocityCommand extends CommandBase {
  private Flywheel flywheel;
  private DrivetrainSubsystem drivetrain;
  private double velocity;

  /**
   * Constructs a new LimelightAlignWithGyroCommand object.
   *
   * @param flywheel the flywheel subsystem this command will control
   * @param drivetrainSubsystem the drivetrain subsystem used to get the distance to the hub
   */
  public LimelightSetFlywheelVelocityCommand(
      Flywheel flywheel, DrivetrainSubsystem drivetrainSubsystem) {
    this.flywheel = flywheel;
    this.drivetrain = drivetrainSubsystem;
    addRequirements(this.flywheel);
  }

  /**
   * This method is invoked once when this command is scheduled. This command assumes that the robot
   * is not in motion as it only queries the distance to the hub when initialized.
   */
  @Override
  public void initialize() {
    // either move this code into execute or move the code in execute here
    this.velocity = this.drivetrain.getVelocityFromLimelight();
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly sets the
   * setpoint of the flywheel velocity to the desired velocity.
   */
  @Override
  public void execute() {
    flywheel.setVelocity(this.velocity);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It does not stops the
   * motion of the flywheel.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * velocity of the flywheel has reached the specified setpoint.
   */
  @Override
  public boolean isFinished() {
    return flywheel.isAtSetpoint();
  }
}
