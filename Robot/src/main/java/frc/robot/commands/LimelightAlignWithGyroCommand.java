package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This command, when executed, instructs the drivetrain subsystem to rotate to aim at the hub. This
 * command initially uses the Limelight to determine the offset and then repeatedly uses the gyro to
 * eliminate that initial offset. The advantage of this approach compared to using the Limelight
 * repeatedly is that the Limelight has more noise than the gyro.
 *
 * <p>Requires: the drivetrain subsystem
 *
 * <p>Finished When: the shooter is aimed
 *
 * <p>At End: stops the drivetrain
 */
public class LimelightAlignWithGyroCommand extends CommandBase {

  private PIDController controller;
  private Drivetrain drivetrain;
  private double setpoint;

  /**
   * Constructs a new LimelightAlignWithGyroCommand object.
   *
   * @param drivetrain the drivetrain subsystem this command will control
   */
  public LimelightAlignWithGyroCommand(Drivetrain drivetrain) {
    this.controller =
        new PIDController(
            DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0, LOOP_PERIOD_SECS);
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets the PID controller for
   * the drivetrain rotation and determines the setpoint (in degrees) of the gyro based on the
   * Limelight and the current gyro value. It is critical that this initialization occurs in this
   * method and not the constructor as this command is constructed once when the RobotContainer is
   * created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    controller.reset();
    double gyro = drivetrain.getGyroscopeRotation().getDegrees();
    double tx = drivetrain.getLimelightX();
    this.setpoint = gyro - tx;
    drivetrain.setGyroSetpoint(this.setpoint);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly obtains the
   * rotational velocity from the PID controller and instructs the drivetrain subsytem to rotate to
   * aligned to the hub.
   */
  @Override
  public void execute() {
    double output = controller.calculate(drivetrain.getGyroscopeRotation().getDegrees(), setpoint);

    drivetrain.aim(0, 0, output); // PID output (rad/s)
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
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when the
   * drivetrain is aimed based on the gyro.
   */
  @Override
  public boolean isFinished() {
    return drivetrain.isAimedWithGyro();
  }
}
