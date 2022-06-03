package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

/**
 * This command, when executed, waits for all cargo to be shot and then stops the motion of the
 * storage and flywheel and disables the drivetrain's x-stance.
 * 
 * Requires: the storage, flywheel, and drivetrain subsystems
 * Finished When: all cargo has been shot
 * At End: stops the motion of the storage and flywheel and disables the drivetrain's x-stance
 */
public class WaitForShotCommand extends CommandBase {
    private final Storage storage;
    private final Flywheel flywheel;
    private final DrivetrainSubsystem drivetrain;
    private int iterations;

    /**
     * Constructs a new WaitForShotCommand object.
     * 
     * @param storage the storage subsystem this command will control
     * @param flywheel the flywheel subsystem this command will control
     * @param drivetrain the drivetrain subsystem this command will control
     */
    public WaitForShotCommand(Storage storage, Flywheel flywheel, DrivetrainSubsystem drivetrain) {
        this.storage = storage;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        addRequirements(storage);
        addRequirements(flywheel);
        addRequirements(drivetrain);
    }

    /**
     * This method is invoked once when this command is scheduled. It initializes the iterations
     * delay counter. It is critical that this initialization occurs in this method and not the
     * constructor as this command is constructed once when the RobotContainer is created, but this
     * method is invoked each time this command is scheduled.
     */
    @Override
    public void initialize() {
        iterations = 0;
    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the
     * stops the motion of the storage and flywheel and disables the drivetrain's x-stance.
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
     * desired number of additional iterations have occurred to ensure the cargo has completely
     * exited the shooter before the flywheel and storage are stopped.
     */
   @Override
    public boolean isFinished() {
        if(storage.isCollectorSensorUnblocked() && storage.isShooterSensorUnblocked()) {
             if(iterations > StorageConstants.WAIT_FOR_SHOT_DELAY) {
                return true;
             }

             iterations++;
        }
        // if cargo is detected at either end of the storage system, reset the iterations delay
        //  counter. This is critical as there is a brief period of time where cargo can be between
        //  the sensors and the iteration counter can be incremented prematurely.
        else {
            iterations = 0;
        }

        return false;
    }
}
