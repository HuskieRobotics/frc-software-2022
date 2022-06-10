package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.storage.Storage;

/**
 * This command, when executed, waits for all cargo to be shot and then stops the motion of the
 * storage and flywheel and disables the drivetrain's x-stance.
 *
 * <p>Requires: the storage, flywheel, and drivetrain subsystems
 *
 * <p>Finished When: all cargo has been shot
 *
 * <p>At End: stops the motion of the storage and flywheel and disables the drivetrain's x-stance
 */
public class WaitForShotCommand extends CommandBase {
  private final Storage storage;
  private final Flywheel flywheel;
  private final Drivetrain drivetrain;
  private int iterationDelayCount;

  /**
   * Constructs a new WaitForShotCommand object.
   *
   * @param storage the storage subsystem this command will control
   * @param flywheel the flywheel subsystem this command will control
   * @param drivetrain the drivetrain subsystem this command will control
   */
  public WaitForShotCommand(Storage storage, Flywheel flywheel, Drivetrain drivetrain) {
    this.storage = storage;
    this.flywheel = flywheel;
    this.drivetrain = drivetrain;
    addRequirements(storage);
    addRequirements(flywheel);
    addRequirements(drivetrain);
  }

  /**
   * This method is invoked once when this command is scheduled. It initializes the
   * iterationDelayCount delay counter. It is critical that this initialization occurs in this
   * method and not the constructor as this command is constructed once when the RobotContainer is
   * created, but this method is invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    iterationDelayCount = 0;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the stops
   * the motion of the storage and flywheel and disables the drivetrain's x-stance.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    this.flywheel.stopFlywheel();
    this.storage.disableStorage();
    this.drivetrain.disableXstance();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when
   * cargo is detected at neither the shooter end of the storage nor the collector end, and the
   * desired number of additional iterationDelayCount have occurred to ensure the cargo has
   * completely exited the shooter before the flywheel and storage are stopped.
   */
  @Override
  public boolean isFinished() {
    if (storage.isCollectorSensorUnblocked() && storage.isShooterSensorUnblocked()) {
      if (iterationDelayCount > StorageConstants.WAIT_FOR_SHOT_DELAY) {
        return true;
      }

      iterationDelayCount++;
    }
    // if cargo is detected at either end of the storage system, reset the iterationDelayCount delay
    //  counter. This is critical as there is a brief period of time where cargo can be between
    //  the sensors and the iteration counter can be incremented prematurely.
    else {
      iterationDelayCount = 0;
    }

    return false;
  }
}
