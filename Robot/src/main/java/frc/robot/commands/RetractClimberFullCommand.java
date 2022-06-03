package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/**
 * This command, when executed, retracts the climber completely.
 *
 * <p>Requires: the elevator subsystem
 *
 * <p>Finished When: the climber is fully retracted
 *
 * <p>At End: stops the elevator
 */
public class RetractClimberFullCommand extends CommandBase {
  private final Elevator m_elevator;

  /**
   * Constructs a new RetractClimberFullCommand object.
   *
   * @param subsystem the elevator subsystem this command will control
   */
  public RetractClimberFullCommand(Elevator subsystem) {
    m_elevator = subsystem;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  /**
   * This method will be invoked every iteration of the Command Scheduler. It sets the setpoint of
   * the elevator position to fully retracted.
   */
  @Override
  public void execute() {
    // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
    //  method instead of repeatedly in this method as well as the following line of code
    m_elevator.setElevatorMotorPosition(ElevatorConstants.MIN_ELEVATOR_ENCODER_HEIGHT, true);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the elevator.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopElevator();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * elevator has reached the specified setpoint, which is fully retracted.
   */
  @Override
  public boolean isFinished() {
    // the responsibility for checking if elevator control is enabled is currently split
    //  between the commands and the elevator subsystem. It should be in a single class;
    //  probably, the elevator subsystem.
    if (!m_elevator.isElevatorControlEnabled()) {
      return true;
    }
    return m_elevator.atSetpoint();
  }
}
