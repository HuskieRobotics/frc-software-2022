package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

/**
 * This command, when executed, retracts the climber just enough to pull the secondary arms off of
 * the previous rung such that the robot is transferred to the next rung.
 *
 * <p>Requires: the elevator subsystem
 *
 * <p>Finished When: the climber reaches the specified position
 *
 * <p>At End: stops the elevator
 */
public class RetractClimberMinimumCommand extends CommandBase {
  private final Elevator elevator;
  private double encoderSetpoint;

  /**
   * Constructs a new RetractClimberMinimumCommand object.
   *
   * @param setpoint the desired position of the elevator in units of encoder ticks
   * @param elevator the elevator subsystem this command will control
   */
  public RetractClimberMinimumCommand(double setpoint, Elevator elevator) {
    this.elevator = elevator;
    this.encoderSetpoint = setpoint;
    addRequirements(elevator);
  }

  /**
   * This method is invoked once when this command is scheduled. It sets the setpoint of the
   * elevator position to specified value. It is critical that this initialization occurs in this
   * method and not the constructor as this command is constructed once when the RobotContainer is
   * created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    elevator.setElevatorMotorPosition(encoderSetpoint, true);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the elevator.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * elevator has reached the specified setpoint.
   */
  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }
}
