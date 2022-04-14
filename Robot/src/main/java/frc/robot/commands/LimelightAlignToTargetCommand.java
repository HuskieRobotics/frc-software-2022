package frc.robot.commands;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignToTargetCommand extends PIDCommand {

    private double tolerance;
    private DrivetrainSubsystem drivetrainSubsystem;

    public LimelightAlignToTargetCommand(double aimTolerance, DrivetrainSubsystem subsystem){
        super(
            new PIDController(DrivetrainConstants.LIMELIGHT_P,DrivetrainConstants.LIMELIGHT_I,0),
            subsystem :: getLimelightX,
            0,
            output -> subsystem.aim(0,0,output),
            subsystem
        );

        this.tolerance = aimTolerance;
        drivetrainSubsystem = subsystem;
    }

    public void initialize() {  // FIXME: remove after tuning
        super.initialize();
        getController().setP(DrivetrainConstants.LIMELIGHT_P);
        getController().setI(DrivetrainConstants.LIMELIGHT_I);
    }

    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        super.end(interrupted);
    }

    public boolean isFinished() {
        return drivetrainSubsystem.isAimed(this.tolerance);
    }
    
}
