package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightMath;
import frc.robot.subsystems.Flywheel;

public class SetFlywheelVelocityCommand extends CommandBase {
    private Flywheel flywheel;
    private double velocity;

    // public SetFlywheelVelocityCommand(Flywheel flywheel, LimelightMath limelight) {
    //     this.flywheel = flywheel;
    //     // FIXME: this will get the ideal velocity when the command is constructed; not
    //     // when the command is scheduled
    //     // The next line should be moved to the initialzie method.
    //     this.velocity = limelight.getIdealVelocity();
    //     addRequirements(this.flywheel);

    // }

    public SetFlywheelVelocityCommand(Flywheel flywheel, double velocity) {
        this.flywheel = flywheel;
        this.velocity = velocity;
        addRequirements(this.flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        flywheel.setVelocity(this.velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return flywheel.isAtSetpoint();
    }
}