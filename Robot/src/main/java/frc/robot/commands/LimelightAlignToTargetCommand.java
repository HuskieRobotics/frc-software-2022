package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

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

  private Drivetrain drivetrain;

  /**
   * Constructs a new LimelightAlignToTargetCommand object.
   *
   * @param drivetrain the drivetrain subsystem this command will control
   */
  public LimelightAlignToTargetCommand(Drivetrain drivetrain) {
    // the input to the rotational PID controller is the number of degrees between the rotation
    //  of the drivetrain and the center of the hub target; the setpoint is 0 (aimed perfectly)
    super(
        new PIDController(
            DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0, LOOP_PERIOD_SECS),
        drivetrain::getLimelightX,
        0,
        output -> drivetrain.aim(0, 0, output),
        drivetrain);

    this.drivetrain = drivetrain;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    super.end(interrupted);
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * drivetrain is aimed based on the Limelight.
   */
  @Override
  public boolean isFinished() {
    return drivetrain.isAimed();
  }
}
