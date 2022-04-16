
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RetractClimberMinimumWithPitchCommand extends CommandBase {
    private final Elevator m_elevator;

    public RetractClimberMinimumWithPitchCommand(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setElevatorSetpoint(ElevatorConstants.LATCH_NEXT_RUNG_ENCODER_HEIGHT);
    }

    @Override
    public void execute() {
        if(m_elevator.isNearLocalMaximum()) {
            m_elevator.setElevatorMotorPosition(ElevatorConstants.LATCH_NEXT_RUNG_ENCODER_HEIGHT, true);
        }
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
