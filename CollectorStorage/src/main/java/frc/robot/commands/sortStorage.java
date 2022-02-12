package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * This command continuously runs the belt in the feeder until cargo is detected at the shooter-end of the feeder, at which
 *  point it stops the belt. If interrupted, this command stops the belt.
 */
public class sortStorage extends CommandBase{
    private Storage storage;
    public sortStorage(Storage storage) {
        this.storage = storage;

        this.addRequirements(this.storage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!this.storage.shooterSensorUnblocked()){
            this.storage.setStoragePower(0);
        }
        else if(!this.storage.collectorSensorUnblocked() & this.storage.shooterSensorUnblocked()){
            this.storage.setStoragePower(.3); //change later make into constant
        }
       
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.storage.setStoragePower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !this.storage.collectorSensorUnblocked() & !this.storage.shooterSensorUnblocked();
    }
}