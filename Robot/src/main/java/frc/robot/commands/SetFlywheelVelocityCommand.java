package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

/**
 * This command, when executed, instructs the flywheel subsystem to spin the flywheel at the
 * specified velocity. It does not stop the flywheel when it ends as it is intended to be used as
 * part of sequence of commands and will be followed by a command to shoot the cargo.
 *
 * <p>Requires: the flywheel subsystem
 *
 * <p>Finished When: the flywheel velocity is at the setpoint
 *
 * <p>At End: leaves the flywheel spinning
 */
public class SetFlywheelVelocityCommand extends CommandBase {
  private Flywheel flywheel;
  private double velocity;

  /**
   * Constructs a new SetFlywheelVelocityCommand object.
   *
   * @param flywheel the flywheel subsystem this command will control
   * @param velocity the velocity setpoint in units of encoder ticks per 100 ms
   */
  public SetFlywheelVelocityCommand(Flywheel flywheel, double velocity) {
    this.flywheel = flywheel;
    this.velocity = velocity;
    addRequirements(this.flywheel);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly sets the
   * setpoint of the flywheel velocity to the desired velocity.
   */
  @Override
  public void execute() {
    // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
    //  method instead of repeatedly in this method as well as the following line of code
    flywheel.setVelocity(this.velocity);
  }

  // Called once the command ends or is interrupted.
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
