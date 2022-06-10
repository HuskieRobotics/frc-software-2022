package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;

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
  private Drivetrain drivetrain;

  /**
   * Constructs a new LimelightAlignWithGyroCommand object.
   *
   * @param flywheel the flywheel subsystem this command will control
   * @param drivetrainSubsystem the drivetrain subsystem used to get the distance to the hub
   */
  public LimelightSetFlywheelVelocityCommand(Flywheel flywheel, Drivetrain drivetrainSubsystem) {
    this.flywheel = flywheel;
    this.drivetrain = drivetrainSubsystem;
    addRequirements(this.flywheel);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly sets the
   * setpoint of the flywheel velocity to the desired velocity.
   */
  @Override
  public void execute() {
    flywheel.setVelocity(this.drivetrain.getVelocityFromLimelight());
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * velocity of the flywheel has reached the specified setpoint.
   */
  @Override
  public boolean isFinished() {
    return flywheel.isAtSetpoint();
  }
}
