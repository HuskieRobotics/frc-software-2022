package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This command, when executed, instructs the drivetrain subsystem to rotate to aim at the hub. This
 * command repeatedly uses the Limelight to aim. The superclass' execute method invokes the
 * drivetrain subsystem's aim method to rotate the drivetrain.
 *
 * <p>Requires: the drivetrain subsystem (handled by the superclass)
 *
 * <p>Finished When: the shooter is aimed
 *
 * <p>At End: stops the drivetrain
 */
public class LimelightAlignToTargetCommand extends PIDCommand {

  private DrivetrainSubsystem drivetrainSubsystem;

  /**
   * Constructs a new LimelightAlignToTargetCommand object.
   *
   * @param subsystem the drivetrain subsystem this command will control
   */
  public LimelightAlignToTargetCommand(DrivetrainSubsystem subsystem) {
    // the input to the rotational PID controller is the number of degrees between the rotation
    //  of the drivetrain and the center of the hub target; the setpoint is 0 (aimed perfectly)
    super(
        new PIDController(DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0),
        subsystem::getLimelightX,
        0,
        output -> subsystem.aim(0, 0, output),
        subsystem);

    drivetrainSubsystem = subsystem;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
    super.end(interrupted);
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * drivetrain is aimed based on the Limelight.
   */
  public boolean isFinished() {
    return drivetrainSubsystem.isAimed();
  }
}
