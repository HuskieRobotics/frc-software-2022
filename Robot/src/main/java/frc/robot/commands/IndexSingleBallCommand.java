package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.storage.Storage;

/**
 * This command, when executed, runs the storage in the reverse direction to ensure that the
 * collected cargo is not in contact with the flywheel.
 *
 * <p>Requires: the storage subsystem
 *
 * <p>Finished When: the command has executed for the desired number of iterations to ensure the
 * cargo is not in contact with the flywheel
 *
 * <p>At End: stops the storage
 */
public class IndexSingleBallCommand extends CommandBase {
  private Storage storage;
  private int indexingDelayCount;

  /**
   * Constructs a new IndexSingleBallCommand object.
   *
   * @param storage the storage subsystem required by this command
   */
  public IndexSingleBallCommand(Storage storage) {
    this.storage = storage;
    this.addRequirements(this.storage);
  }

  /**
   * This method is invoked once when this command is scheduled. It sets teh motor of the storage to
   * the outtake power and initializes the indexing delay counter. It is critical that this
   * initialization occurs in this method and not the constructor as this command is constructed
   * once when the RobotContainer is created, but this method is invoked each time this command is
   * scheduled.
   */
  @Override
  public void initialize() {
    indexingDelayCount = 0;
    storage.setStoragePower(StorageConstants.OUTTAKE_POWER);
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler. It repeatedly increments
   * the indexing delay counter.
   */
  @Override
  public void execute() {
    indexingDelayCount++;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the storage.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    storage.disableStorage();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when
   * this command has executed for the desired number of iterations.
   */
  @Override
  public boolean isFinished() {
    return indexingDelayCount > StorageConstants.INDEXING_BACKWARD_DURATION;
  }
}
