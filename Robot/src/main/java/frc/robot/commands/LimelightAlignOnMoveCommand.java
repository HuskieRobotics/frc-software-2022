package frc.robot.commands;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignOnMoveCommand extends PIDCommand {

    private DrivetrainSubsystem drivetrainSubsystem;

    public LimelightAlignOnMoveCommand(DrivetrainSubsystem subsystem){
        super(
            new PIDController(DrivetrainConstants.LIMELIGHT_P,DrivetrainConstants.LIMELIGHT_I,0),
            subsystem :: getLimelightX,
            0,
            (output) -> {
                int AIMING_LINEAR_SPEED_MULTIPLIER = 1;
                // CHECK IF isLimelightTargetVisible WORKS
                if (drivetrainSubsystem.isLimelightTargetVisible()) { //if the target is visible, try to aim with PID
                    subsystem.aim(
                        () -> -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * AIMING_LINEAR_SPEED_MULTIPLIER, //joystick x
                        () -> -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * AIMING_LINEAR_SPEED_MULTIPLIER, //joystick y
                        output //PID output (rad/s)
                    );
                } else { //if we cant see the target, let ian rotate
                    getController().reset();    // if we don't see the target, reset the PID controller so it doesn't get wound up
                    subsystem.drive(
                        () -> -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, //joystick x
                        () -> -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, //joystick y
                        () -> -modifyAxis(joystick1.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) //joystick rotation
                    );
                }
                
            },
            subsystem
        );

        drivetrainSubsystem = subsystem;
    }

    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        super.end(interrupted);
    }

    public boolean isFinished() {
        return drivetrainSubsystem.isAimed();
    }
    
}
