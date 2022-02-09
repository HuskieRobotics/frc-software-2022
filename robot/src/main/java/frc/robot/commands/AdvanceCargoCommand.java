package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class AdvanceCargoCommand extends CommandBase{
    private Feeder feeder;
    
    public AdvanceCargoCommand(Feeder feeder) {
        this.feeder = feeder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.feeder.advanceBelt();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.feeder.stopBelt();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.feeder.isCargoAdvanced();
    }
}

