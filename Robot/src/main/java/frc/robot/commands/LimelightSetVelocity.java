package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightMath;
import frc.robot.subsystems.Flywheel;

public class LimelightSetVelocity extends CommandBase {
    private Flywheel flywheel;
    private double velocity;
    private LimelightMath limelightMath;

    public LimelightSetVelocity(Flywheel fw, LimelightMath lm) {
        flywheel = fw;
        limelightMath = lm;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        velocity = limelightMath.getIdealVelocity();
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
