package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotGlobal;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldRelativeDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public FieldRelativeDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        ShuffleboardTab tab2 = Shuffleboard.getTab("Drivetrain");
        m_drivetrainSubsystem.setFieldRelative(true);
        System.out.println("Field Relative");
        RobotGlobal.DriveState = "FieldRelative";
        SmartDashboard.putString("Drivestate", RobotGlobal.DriveState);
    }
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement

            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

        

        
       

        

        
        
        //m_drivetrainSubsystem.drive(new ChassisSpeeds(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
