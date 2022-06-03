package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

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
   * @param subsystem the elevator subsystem this command will control
   */
  public RetractClimberMinimumCommand(double setpoint, Elevator subsystem) {
    elevator = subsystem;
    encoderSetpoint = setpoint;
    addRequirements(elevator);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It sets the setpoint of
   * the elevator to the specified position.
   */
  @Override
  public void execute() {
    // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
    //  method instead of repeatedly in this method as well as the following line of code
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
    // the responsibility for checking if elevator control is enabled is currently split
    //  between the commands and the elevator subsystem. It should be in a single class;
    //  probably, the elevator subsystem.
    if (!elevator.isElevatorControlEnabled()) {
      return true;
    }
    return elevator.atSetpoint();
  }
}
