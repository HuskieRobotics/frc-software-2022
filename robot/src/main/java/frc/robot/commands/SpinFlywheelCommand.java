package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class SpinFlywheelCommand extends CommandBase{
    private Flywheel flywheel;
    private double velocity;
    
    public SpinFlywheelCommand(Flywheel flywheel, double velocity) {
        this.flywheel = flywheel;
        this.velocity = velocity;
        
        this.addRequirements(this.flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.flywheel.setVelocity(this.velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.flywheel.isAtSetpoint();
    }
}

