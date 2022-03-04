package frc.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RetractClimberFullCommand extends CommandBase {
    private final Elevator m_elevator;


    public RetractClimberFullCommand(Elevator subsystem){ 
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPosition(ElevatorConstants.MIN_ELEVATOR_ENCODER_HEIGHT);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint() && m_elevator.atSetpoint();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}