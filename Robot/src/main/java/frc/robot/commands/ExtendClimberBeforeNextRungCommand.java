package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondaryArm;

public class ExtendClimberBeforeNextRungCommand extends CommandBase {
    private final Elevator m_elevator;
    private final SecondaryArm m_secondMechanism;

    // FIXME: delete is unused
    public ExtendClimberBeforeNextRungCommand(Elevator elevator, SecondaryArm secondaryArm) {
        m_elevator = elevator;
        m_secondMechanism = secondaryArm;
        addRequirements(m_elevator);
        addRequirements(m_secondMechanism);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPosition(ElevatorConstants.REACH_JUST_BEFORE_NEXT_RUNG, true);

        // if the robot has been transferred from the elevator to secondary arms, move the secondary arms in to dampen the swing
        if(m_elevator.hasTransferredToSecondary()) {
            m_secondMechanism.moveSecondaryArmIn();
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
