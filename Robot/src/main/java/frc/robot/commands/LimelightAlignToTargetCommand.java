package frc.robot.commands;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignToTargetCommand extends PIDCommand {

    private DrivetrainSubsystem drivetrainSubsystem;

    public LimelightAlignToTargetCommand(DrivetrainSubsystem subsystem){
        super(
            new PIDController(DrivetrainConstants.LIMELIGHT_P,0,0),
            subsystem :: getLimelightX,
            0,
            output -> subsystem.aim(0,0,output),//0.7
            subsystem
        );

        drivetrainSubsystem = subsystem;
    }

    public void initialize() {  // FIXME: remove after tuning
        super.initialize();
        getController().setP(DrivetrainConstants.LIMELIGHT_P);
    }

    public void end(boolean interrupted) {
        drivetrainSubsystem.resetCenterGrav();
        super.end(interrupted);
    }

    public boolean isFinished() {
        return drivetrainSubsystem.isAimed();
    }
    
}
