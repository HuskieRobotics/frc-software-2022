package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondaryArm;

/**
 * This command, when executed, extends the climber below and slightly beyond the next rung (high or
 * traverse). It uses the pitch of the robot to determine when the robot's swing is at a local
 * minimum such that the climber will clear under the rung and not into or over it. This is
 * essential such that the hooks are positioned properly to grab the rung. It is intended to be
 * invoked when the climber is fully retracted and after the secondary arms are moved outward and
 * before the climber is retracted the minimum amount to transfer to the next rung.
 * 
 * Requires: the elevator and secondary arm subsystems
 * Finished When: the climber is positioned slightly beyond the next rung
 * At End: stops the elevator
 */
public class ReachToNextRungWithPitchCommand extends CommandBase {
    private final Elevator m_elevator;
    private final SecondaryArm m_secondMechanism;
    private boolean startedFinalExtension;

    /**
     * Constructs a new ReachToNextRungWithPitchCommand object.
     * 
     * @param elevator the elevator subsystem this command will control
     * @param secondaryArm the secondary arm subsystem this command will control
     */
    public ReachToNextRungWithPitchCommand(Elevator elevator, SecondaryArm secondaryArm) {
        m_elevator = elevator;
        m_secondMechanism = secondaryArm;
        addRequirements(m_elevator);
        addRequirements(m_secondMechanism);
    }

    /**
     * This method is invoked once when this command is scheduled. It initializes the state
     * variable that tracks if cimber has started moving under the rung. It is critical that this
     * initialization occurs in this method and not the constructor as this command is constructed
     * once when the RobotContainer is created, but this method is invoked each time this command
     * is scheduled.
     */
    @Override
    public void initialize() {
        this.startedFinalExtension = false;
    }

    /**
     * This method will be invoked every iteration of the Command Scheduler. It repeatedly checks
     * if the climber is near the next rung. If it is, and if the robot's swing is at a local
     * minimum, it extends under and slightly beyond the rung. If the climber is near the next
     * rung, but not at a local minimum, it stops the climber until it is.
     */
    @Override
    public void execute() {
        // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
        //  method instead of repeatedly in this method as well as the following line of code
        m_elevator.setElevatorMotorPosition(ElevatorConstants.REACH_TO_NEXT_RUNG_HEIGHT, true);

        // if the robot has been transferred from the elevator to secondary arms, move the
        //  secondary arms in to dampen the swing
        m_secondMechanism.moveSecondaryArmIn();

        if(m_elevator.isApproachingNextRung()) {
            if(m_elevator.isNearLocalMinimum()) {
                this.startedFinalExtension = true;
                m_elevator.setElevatorMotorPosition(ElevatorConstants.REACH_TO_NEXT_RUNG_HEIGHT, true);
            }
            // once the elevator has started extending under the next rung, don't stop it based on
            //  the robot's swing
            else if(!this.startedFinalExtension) {
                m_elevator.stopElevator();
            }
        }
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
     * the elevator has reached the specified setpoint, which is slightly beyond the next rung.
     */
    @Override
    public boolean isFinished() {
        // the responsibility for checking if elevator control is enabled is currently split
        //  between the commands and the elevator subsystem. It should be in a single class;
        //  probably, the elevator subsystem.
        if (!m_elevator.isElevatorControlEnabled()) {
            return true;
        }

        // the order of these methods is critical in that the isContactingUnderRung method
        //      must be invoked before the elevator reaches the set point
        return m_elevator.atSetpoint();
    }

}
