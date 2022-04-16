package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;

public class LimelightAlignOnMoveCommand extends CommandBase {

    private PIDController controller;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Flywheel flywheelSubsystem;
    private Storage storageSubsystem;
    private Joystick joystick0;
    private Joystick joystick1;

    public LimelightAlignOnMoveCommand(DrivetrainSubsystem drivetrain, Flywheel flywheel, Storage storage, Joystick joystick0,
            Joystick joystick1) {
        controller = new PIDController(DrivetrainConstants.LIMELIGHT_P, DrivetrainConstants.LIMELIGHT_I, 0);
        drivetrainSubsystem = drivetrain;
        flywheelSubsystem = flywheel;
        storageSubsystem = storage;
        this.joystick0 = joystick0;
        this.joystick1 = joystick1;

        addRequirements(drivetrainSubsystem);
        addRequirements(flywheelSubsystem);
        // don't add the storage as a requirement as we are only determining if cargo is indexed
    }

    @Override
    public void initialize() {
        controller.reset();
        drivetrainSubsystem.enableAutoAimAndShoot();
    }

    @Override
    public void execute() {
        double output = controller.calculate(drivetrainSubsystem.getLimelightX(), 0);

        final int AIMING_LINEAR_SPEED_MULTIPLIER = 1;
        if (drivetrainSubsystem.isLimelightTargetVisible()) { // if the target is visible, try to aim with PID

            double tx = Math.toRadians(drivetrainSubsystem.getLimelightX()); //limelight offset angle in rad, left of camera is negative angle
            // positive limelight distance in meters; add the distance from the limelight to the center of the robot
            //  and the distance for the retroreflective tape to the center of the hub before converting to meters
            double d = (drivetrainSubsystem.getLimelightDistanceIn() + LimelightConstants.EDGE_TO_CENTER_HUB_DISTANCE) * 0.0254;
            // assuming robot v is forward/left positive and in the direction of the collector
            double dhdt = drivetrainSubsystem.getVelocityX();
            double dldt = drivetrainSubsystem.getVelocityY();
            double h = d * Math.cos(tx); // h is always positive
            double l = -d * Math.sin(tx); // positive if the target is to the left of the robot

            double w = -(l * dhdt - h * dldt) / (d*d); // negative dtheta/dt, assuming positive ccw

            drivetrainSubsystem.aim(
                    -RobotContainer.modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                            * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick x
                    -RobotContainer.modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                            * AIMING_LINEAR_SPEED_MULTIPLIER, // joystick y
                    output + w // PID output (rad/s)
            );

            // only turn on the flywheel if there is a cargo indexed
            if(!storageSubsystem.isShooterSensorUnblocked()) {
                flywheelSubsystem.setVelocity(drivetrainSubsystem.getVelocityFromLimelight());
            }

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
        drivetrainSubsystem.disableAutoAimAndShoot();
    }

    @Override
    public boolean isFinished() {
        // shooter is aimed AND flywheel is at speed AND robot is within optimal shooting distance
        return drivetrainSubsystem.isLimelightTargetVisible() &&
                drivetrainSubsystem.isAimed() && flywheelSubsystem.isAtSetpoint() &&
                (drivetrainSubsystem.getLimelightDistanceIn() < LimelightConstants.AUTO_SHOT_HUB_FAR_DISTANCE && 
                drivetrainSubsystem.getLimelightDistanceIn() > LimelightConstants.AUTO_SHOT_HUB_CLOSE_DISTANCE);
    }

}
