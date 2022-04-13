package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

public class WaitForShotCommand extends CommandBase {
    private final Storage storage;
    private final Flywheel flywheel;
    private final DrivetrainSubsystem drivetrain;
    private int iterations;

    public WaitForShotCommand(Storage storage, Flywheel flywheel, DrivetrainSubsystem drivetrain) {
        this.storage = storage;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        addRequirements(storage);
        addRequirements(flywheel);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        iterations = 0;
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheel.stopFlywheel();
        this.storage.disableStorage();
        this.drivetrain.disableXstance();
    }

    @Override
    public boolean isFinished() {
        if(storage.isCollectorSensorUnblocked() && storage.isShooterSensorUnblocked()) {
             if(iterations > StorageConstants.WAIT_FOR_SHOT_DELAY) {
                return true;
             }

             iterations++;
        }
        else {
            iterations = 0;
        }

        return false;
    }
}
