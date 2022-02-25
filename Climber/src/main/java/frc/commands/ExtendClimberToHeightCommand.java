package frc.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondMechanism;


public class ExtendClimberToHeightCommand extends CommandBase {
    private final Elevator m_elevator;
    private double m_height;


    public ExtendClimberToHeightCommand(Elevator subsystem){ 
        m_elevator = subsystem;
        addRequirements(m_elevator);
        m_height = ElevatorConstants.MIN_ELEVATOR_ENCODER_HEIGHT;
    }

    public ExtendClimberToHeightCommand(Elevator subsystem, double height){ 
        m_elevator = subsystem;
        addRequirements(m_elevator);
        m_height = height;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPosition(m_height);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_elevator.atLeftSetpoint();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
