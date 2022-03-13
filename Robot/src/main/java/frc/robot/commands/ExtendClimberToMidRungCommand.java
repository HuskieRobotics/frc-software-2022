package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ExtendClimberToMidRungCommand extends CommandBase {
    private final Elevator m_elevator;

    public ExtendClimberToMidRungCommand(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPosition(ElevatorConstants.MID_RUNG_HEIGHT);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.disableElevator();

    }

    @Override
    public boolean isFinished() {
        if (!m_elevator.isElevatorControlEnabled()) {
            return true;
        }
        return m_elevator.atSetpoint();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
