package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class RetractClimberMinimumWithPitchCommand extends CommandBase {
  private final Elevator elevator;

  public RetractClimberMinimumWithPitchCommand(Elevator subsystem) {
    elevator = subsystem;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setElevatorSetpoint(ElevatorConstants.LATCH_HIGH_RUNG_ENCODER_HEIGHT);
  }

  @Override
  public void execute() {
    if (elevator.isNearLocalMaximum()) {
      elevator.setElevatorMotorPosition(ElevatorConstants.LATCH_HIGH_RUNG_ENCODER_HEIGHT, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  @Override
  public boolean isFinished() {
    if (!elevator.isElevatorControlEnabled()) {
      return true;
    }
    return elevator.atSetpoint();
  }
}
