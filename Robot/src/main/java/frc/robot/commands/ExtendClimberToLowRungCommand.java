package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/**
 * This command, when executed, extends the climber slightly above the low rung in preparation to
 * climb the low rung.
 *
 * <p>Requires: the Elevator subsystem
 *
 * <p>Finished When: the climber is positioned slightly above the low rung
 *
 * <p>At End: stops the elevator
 */
public class ExtendClimberToLowRungCommand extends CommandBase {
  private final Elevator elevator;

  /**
   * Constructs a new ExtendClimberToLowRungCommand object.
   *
   * @param subsystem the elevator subsystem this command will control
   */
  public ExtendClimberToLowRungCommand(Elevator subsystem) {
    elevator = subsystem;
    addRequirements(elevator);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It sets the setpoint of
   * the elevator position to slightly above the low rung.
   */
  @Override
  public void execute() {
    // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
    //  method instead of repeatedly in this method
    elevator.setElevatorMotorPosition(ElevatorConstants.LOW_RUNG_HEIGHT, true);
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
   * elevator has reached the specified setpoint, which is slightly above the low rung.
   */
  @Override
  public boolean isFinished() {
    /*
      the responsibility for checking if elevator control is enabled is currently split
      between the commands and the elevator subsystem. It should be in a single class,
      probably, the elevator subsystem.
    */
    if (!elevator.isElevatorControlEnabled()) {
      return true;
    }
    return elevator.atSetpoint();
  }
}
