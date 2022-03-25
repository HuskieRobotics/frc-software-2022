package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

public class WaitForShotCommand extends CommandBase {
    private final Storage storage;
    private int iterations;

    public WaitForShotCommand(Storage storage) {
        this.storage = storage;
        addRequirements(storage);
    }

    @Override
    public void initialize() {
        iterations = 0;
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
