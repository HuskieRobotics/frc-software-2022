package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LimelightMath;
import frc.robot.Constants.*;

public class LimelightVelocity extends CommandBase {
    private Flywheel flywheel;
    private LimelightMath lime;
    private double velo;

    public LimelightVelocity(Flywheel flywheel, LimelightMath lime) {
        this.flywheel = flywheel;
        this.lime = lime;
        addRequirements(this.flywheel); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.velo = FlywheelConstants.SLOPE * this.lime.getLimelightDistanceIn() + FlywheelConstants.YINTERCEPT; 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        flywheel.setVelocity(velo);
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