package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;

public class LimelightAlignOnMoveCommand extends CommandBase {

    private PIDController controller;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Flywheel flywheelSubsystem;
    private Joystick joystick0;
    private Joystick joystick1;

    public LimelightAlignOnMoveCommand(DrivetrainSubsystem drivetrain, Flywheel flywheel, Joystick joystick0,
            Joystick joystick1) {
        controller = new PIDController(DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0);
        drivetrainSubsystem = drivetrain;
        flywheelSubsystem = flywheel;
        this.joystick0 = joystick0;
        this.joystick1 = joystick1;

        addRequirements(drivetrainSubsystem);
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {
        double output = controller.calculate(drivetrainSubsystem.getLimelightX(), 0);

        int AIMING_LINEAR_SPEED_MULTIPLIER = 1;
        // CHECK IF isLimelightTargetVisible WORKS
        if (drivetrainSubsystem.isLimelightTargetVisible()) { // if the target is visible, try to aim with PID
            drivetrainSubsystem.aim(
                    -RobotContainer.modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                            * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick x
                    -RobotContainer.modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                            * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick y
                    output // PID output (rad/s)
            );

            flywheelSubsystem.setVelocity(drivetrainSubsystem.getVelocityFromLimelight());

        } else { // if we cant see the target, let ian rotate
            controller.reset(); // if we don't see the target, reset the PID controller so it doesn't get wound
                                // up
            drivetrainSubsystem.drive(
                    -RobotContainer.modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, // joystick
                                                                                                                       // x
                    -RobotContainer.modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, // joystick
                                                                                                                       // y
                    -RobotContainer.modifyAxis(joystick1.getX())
                            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // joystick
            // rotation
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // shooter is aimed AND flywheel is at speed AND robot is within optimal shooting distance
        return drivetrainSubsystem.isAimed() && flywheelSubsystem.isAtSetpoint() &&
                (drivetrainSubsystem.getLimelightDistanceIn() < LimelightConstants.HUB_WALL_DISTANCE && 
                drivetrainSubsystem.getLimelightDistanceIn() > LimelightConstants.HUB_TARMAC_DISTANCE);
    }

}
