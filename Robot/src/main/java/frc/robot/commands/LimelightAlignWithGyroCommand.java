package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignWithGyroCommand extends CommandBase {

    private PIDController controller;
    private DrivetrainSubsystem drivetrainSubsystem;
    private double setpoint;
    
    public LimelightAlignWithGyroCommand(DrivetrainSubsystem drivetrain) {
        controller = new PIDController(DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0);
        drivetrainSubsystem = drivetrain;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        controller.reset();
        controller.setP(DrivetrainConstants.LIMELIGHT_P);
        controller.setI(DrivetrainConstants.LIMELIGHT_I);
        double gyro = drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        double tx = drivetrainSubsystem.getLimelightX();
        this.setpoint = gyro - tx;
    }

    @Override
    public void execute() {
        double output = controller.calculate(drivetrainSubsystem.getGyroscopeRotation().getDegrees(), setpoint);

        drivetrainSubsystem.aim(0, 0, output); // PID output (rad/s)
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return drivetrainSubsystem.isAimed(DrivetrainConstants.LIMELIGHT_ALIGNMENT_TOLERANCE);
    }

}
