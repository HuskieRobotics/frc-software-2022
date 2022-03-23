package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

/**
 * This command continuously runs the belt in the feeder until cargo is detected
 * at the shooter-end of the feeder, at which
 * point it stops the belt. If interrupted, this command stops the belt.
 */
public class SortStorageCommand extends CommandBase {
    private Storage m_storage;
    private int indexingDelay;

    public SortStorageCommand(Storage storage) {
        this.m_storage = storage;
        this.addRequirements(this.m_storage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        indexingDelay = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!this.m_storage.isShooterSensorUnblocked()) {
            indexingDelay++;
            if(indexingDelay == 8) {
                this.m_storage.setStoragePower(StorageConstants.OUTTAKE_POWER);
            }
            else if(indexingDelay > 8) {
                this.m_storage.disableStorage();
            }
        } else if (!this.m_storage.isCollectorSensorUnblocked() & this.m_storage.isShooterSensorUnblocked()) {
            this.m_storage.enableStorage();
            indexingDelay = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_storage.disableStorage();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !this.m_storage.isCollectorSensorUnblocked() && !this.m_storage.isShooterSensorUnblocked();
    }
}