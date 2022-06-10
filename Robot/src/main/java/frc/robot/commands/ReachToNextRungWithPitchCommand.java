package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.secondary_arm.SecondaryArm;

/**
 * This command, when executed, extends the climber below and slightly beyond the next rung (high or
 * traverse). It uses the pitch of the robot to determine when the robot's swing is at a local
 * minimum such that the climber will clear under the rung and not into or over it. This is
 * essential such that the hooks are positioned properly to grab the rung. It is intended to be
 * invoked when the climber is fully retracted and after the secondary arms are moved outward and
 * before the climber is retracted the minimum amount to transfer to the next rung.
 *
 * <p>Requires: the elevator and secondary arm subsystems
 *
 * <p>Finished When: the climber is positioned slightly beyond the next rung
 *
 * <p>At End: stops the elevator
 */
public class ReachToNextRungWithPitchCommand extends CommandBase {
  private final Elevator elevator;
  private final SecondaryArm secondaryArm;
  private boolean startedFinalExtension;

  /**
   * Constructs a new ReachToNextRungWithPitchCommand object.
   *
   * @param elevator the elevator subsystem this command will control
   * @param secondaryArm the secondary arm subsystem this command will control
   */
  public ReachToNextRungWithPitchCommand(Elevator elevator, SecondaryArm secondaryArm) {
    this.elevator = elevator;
    this.secondaryArm = secondaryArm;
    addRequirements(elevator);
    addRequirements(secondaryArm);
  }

  /**
   * This method is invoked once when this command is scheduled. It initializes the state variable
   * that tracks if cimber has started moving under the rung. It is critical that this
   * initialization occurs in this method and not the constructor as this command is constructed
   * once when the RobotContainer is created, but this method is invoked each time this command is
   * scheduled.
   */
  @Override
  public void initialize() {
    this.startedFinalExtension = false;
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly checks if
   * the climber is near the next rung. If it is, and if the robot's swing is at a local minimum, it
   * extends under and slightly beyond the rung. If the climber is near the next rung, but not at a
   * local minimum, it stops the climber until it is.
   */
  @Override
  public void execute() {
    // it may be more efficient to only invoke setElevatorMotorPosition in the initialize
    //  method instead of repeatedly in this method as well as the following line of code
    elevator.setElevatorMotorPosition(ElevatorConstants.REACH_TO_NEXT_RUNG_HEIGHT, true);

    // if the robot has been transferred from the elevator to secondary arms, move the
    //  secondary arms in to dampen the swing
    secondaryArm.moveSecondaryArmIn();

    if (elevator.isApproachingNextRung()) {
      if (elevator.isNearLocalMinimum()) {
        this.startedFinalExtension = true;
        elevator.setElevatorMotorPosition(ElevatorConstants.REACH_TO_NEXT_RUNG_HEIGHT, true);
      }
      // once the elevator has started extending under the next rung, don't stop it based on
      //  the robot's swing
      else if (!this.startedFinalExtension) {
        elevator.stopElevator();
      }
    }
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
   * elevator has reached the specified setpoint, which is slightly beyond the next rung.
   */
  @Override
  public boolean isFinished() {
    return elevator.atSetpoint();
  }
}
