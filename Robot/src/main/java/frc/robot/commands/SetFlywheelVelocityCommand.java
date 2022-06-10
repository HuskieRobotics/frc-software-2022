package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel.Flywheel;

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
   * This method is invoked once when this command is scheduled. It sets the setpoint of the
   * flywheel velocity to the desired velocity. It is critical that this initialization occurs in
   * this method and not the constructor as this command is constructed once when the RobotContainer
   * is created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    flywheel.setVelocity(this.velocity);
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
