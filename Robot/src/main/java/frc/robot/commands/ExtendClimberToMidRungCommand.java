package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/**
 * This command, when executed, extends the climber slightly above the mid rung in preparation
 * to climb the mid rung.
 * 
 * Requires: the Elevator subsystem
 * Finished When: the climber is positioned slightly above the mid rung
 * At End: stops the elevator
 */
public class ExtendClimberToMidRungCommand extends CommandBase {
    private final Elevator m_elevator;

    /**
     * Constructs a new ExtendClimberToMidRungCommand object.
     * 
     * @param subsystem the elevator subsystem this command will control
     */
    public ExtendClimberToMidRungCommand(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    /**
     * This method will be invoked every iteration of the Command Scheduler. It sets the setpoint
     * of the elevator position to slightly above the mid rung.
     */
    @Override
    public void execute() {
        // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
        //  method instead of repeatedly in this method
        m_elevator.setElevatorMotorPosition(ElevatorConstants.MID_RUNG_HEIGHT, true);
    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the
     * motion of the elevator.
     * 
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
        m_elevator.stopElevator();

    }

    /**
     * This method is invoked at the end of each Command Scheduler iteration. It returns true when
     * the elevator has reached the specified setpoint, which is slightly above the mid rung.
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
