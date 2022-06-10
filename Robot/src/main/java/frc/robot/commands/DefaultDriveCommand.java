package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * A single instance of this command should be created in the RobotContainer class and initialized
 * with the joystick inputs as the suppliers. As a default command, this command will be scheduled
 * whenever no other commands that require the drivetrain subsystem are scheduled.
 *
 * <p>Requires: the Drivetrain subsystem.
 *
 * <p>Finished When: never unless interrupted
 *
 * <p>At End: stops the drivetrain
 */
public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  /**
   * Construct a new DefaultDriveCommand object.
   *
   * @param drivetrain the Drivetrain subsystem required by this command
   * @param translationXSupplier supplies the desired velocity in the x direction (m/s)
   * @param translationYSupplier supplies the desired velocity in the y direction (m/s)
   * @param rotationSupplier supplies the desried rotational velocity (m/s)
   */
  public DefaultDriveCommand(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(this.drivetrain);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. This method should only
   * invoke the drive method on the Drivetrain subsystem. Any other logic needs to be in the drive
   * method as this method is only one of several methods that invoke the drive method.
   */
  @Override
  public void execute() {
    drivetrain.drive(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        rotationSupplier.getAsDouble());
  }

  /**
   * This method will be invoked when this command is interrupted. The stop method of the Drivetrain
   * subsystem needs to be invoked or else the robot will continue to move.
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
