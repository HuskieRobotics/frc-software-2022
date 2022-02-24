package frc.robot.commands;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightAlignToTargetCommand extends PIDCommand {
    public LimelightAlignToTargetCommand(DrivetrainSubsystem m_DrivetrainSubsystem){
        super(
            new PIDController(DrivetrainConstants.P_LIMELIGHT,0,0),
            m_DrivetrainSubsystem :: getLimelightX,
            0,
            output -> m_DrivetrainSubsystem.drive(0,0,output),//0.7
            m_DrivetrainSubsystem
        );
    }
    
}
