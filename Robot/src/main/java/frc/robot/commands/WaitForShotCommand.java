package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;
import static frc.robot.Constants.*;

public class WaitForShotCommand extends CommandBase {
    private final Storage storage;
    private final Flywheel flywheel;

    public WaitForShotCommand(Flywheel flywheel, Storage storage) {
        this.storage = storage;
        this.flywheel = flywheel;
        addRequirements(storage);
        addRequirements(flywheel);
    }

    @Override
    public boolean isFinished() {
        return (flywheel.getVelocity() < (flywheel.getVelocitySetPoint() - FlywheelConstants.SHOT_VELOCITY_DIP));
    }

    @Override
    public void end(boolean interrupted) {
        storage.disableStorage();
    }
}
