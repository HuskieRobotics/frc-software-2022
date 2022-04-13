package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ExtendClimberToLowRungCommand extends CommandBase {
    private final Elevator m_elevator;

    public ExtendClimberToLowRungCommand(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPosition(ElevatorConstants.LOW_RUNG_HEIGHT, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopElevator();

    }

    @Override
    public boolean isFinished() {
        if (!m_elevator.isElevatorControlEnabled()) {
            return true;
        }
        return m_elevator.atSetpoint();
    }

}
