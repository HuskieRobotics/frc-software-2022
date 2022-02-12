package frc.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondMechanism;


public class ExtendClimberToHeight extends CommandBase {
    private final Elevator m_elevator;
    private double m_power;
    private double m_height;


    public ExtendClimberToHeight(Elevator subsystem){ 
        m_elevator = subsystem;
        addRequirements(m_elevator);
        m_power = ElevatorConstants.ELEVATOR_MOTOR_POWER;
        m_height = ElevatorConstants.MIN_ELEVATOR_ENCODER_HEIGHT;
    }

    public ExtendClimberToHeight(Elevator subsystem, double height){ 
        m_elevator = subsystem;
        addRequirements(m_elevator);
        m_power = ElevatorConstants.ELEVATOR_MOTOR_POWER;
        m_height = height;
    }

    @Override
    public void initialize() {
        if (m_elevator.getElevatorHeight() > m_height){
            m_power *= -1;
        }
    }

    @Override
    public void execute() {
        m_elevator.setElevatorMotorPower(m_power);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(m_elevator.getElevatorHeight() == m_height){  //FIX_ME should be changed to a tollerance(can be +- sonme value)
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
