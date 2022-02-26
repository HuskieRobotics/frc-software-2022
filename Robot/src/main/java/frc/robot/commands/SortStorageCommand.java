package frc.robot.commands;

import javax.swing.event.CellEditorListener;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.*;

/**
 * This command continuously runs the belt in the feeder until cargo is detected at the shooter-end of the feeder, at which
 *  point it stops the belt. If interrupted, this command stops the belt.
 */
public class SortStorageCommand extends CommandBase{
    private Storage m_storage;

    public SortStorageCommand(Storage storage) {
        this.m_storage = storage;
        this.addRequirements(this.m_storage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!this.m_storage.isShooterSensorUnblocked()){
            this.m_storage.setStoragePower(0);
        }
        else if(!this.m_storage.isCollectorSensorUnblocked() & this.m_storage.isShooterSensorUnblocked()){
            this.m_storage.setStoragePower(StorageConstants.STORAGE_DEFAULT_SPEED); 
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.m_storage.setStoragePower(0);

        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        //  return !this.m_storage.isCollectorSensorUnblocked() && !this.m_storage.isShooterSensorUnblocked();
    }
}