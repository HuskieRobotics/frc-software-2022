package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;

public class LimelightSetFlywheelVelocity extends CommandBase {
    private Flywheel flywheel;
    private DrivetrainSubsystem drivetrain;
    private double velocity;


    public LimelightSetFlywheelVelocity(Flywheel flywheel, DrivetrainSubsystem drivetrainSubsystem) {
        this.flywheel = flywheel;
        this.drivetrain = drivetrainSubsystem;
        addRequirements(this.flywheel);
    }

    @Override
    public void initialize(){
        this.velocity = this.drivetrain.getVelocityFromLimelight();
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