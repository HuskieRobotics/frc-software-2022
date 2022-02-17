package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.sysid.SysIdDrivetrainLogger;

public class Characterize extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private SysIdDrivetrainLogger logger;   
    private Double prevAngle = 0.0;
    private Double prevTime = 0.0;
    private boolean resetComplete;

    public Characterize(DrivetrainSubsystem subsystem) {
        drivetrainSubsystem = subsystem;
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // reset gyro and encoders
        // set timeperiod to .005
        //drivetrainSubsystem.m_drive.setDeadband(0.0);
        // The following is called for the side-effect of resetting the 
        // DrivetrainSubsystem odometers.
        //drivetrainSubsystem.resetOdometry(drivetrainSubsystem.m_odometry.getPoseMeters()); 
        logger = new SysIdDrivetrainLogger();
        logger.updateThreadPriority();
        logger.initLogging();
        resetComplete = false;
    }
   
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double frontLeftPosition = drivetrainSubsystem.getFrontLeftEncoderPosition();
        double frontLeftRate = drivetrainSubsystem.getFrontLeftEncoderVelocity();
        double frontRightPosition = drivetrainSubsystem.getFrontRightEncoderPosition();
        double frontRightRate = drivetrainSubsystem.getFrontRightEncoderVelocity();
        double backLeftPosition = drivetrainSubsystem.getBackLeftEncoderPosition();
        double backLeftRate = drivetrainSubsystem.getBackLeftEncoderVelocity();
        double backRightPosition = drivetrainSubsystem.getBackRightEncoderPosition();
        double backRightRate = drivetrainSubsystem.getBackRightEncoderVelocity();
        
        double angularPosition = -drivetrainSubsystem.getGyroscopeRotation().getRadians();      // not sure why this is negated....
        double deltaAngle = angularPosition - prevAngle;
        double now = Timer.getFPGATimestamp();
        double deltaTime = now - prevTime;
        double angularRate = prevTime==0 || deltaTime==0 ? 0.0 : deltaAngle/deltaTime;
        prevAngle = angularPosition;
        prevTime = now;

        // Resetting encoders takes non-zero time on CAN-based encoders
        // Wait for the reset to complete
        if (!resetComplete) {
            if (frontLeftPosition > 0.01 || frontRightPosition > 0.01 || backLeftPosition > 0.01 || backRightPosition > 0.01) return;
            resetComplete = true;
        }
        logger.log(frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition, frontLeftRate, frontRightRate, 
        backLeftRate, backRightRate, angularPosition, angularRate);
        drivetrainSubsystem.driveVolts(logger.getMotorVoltage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Characterization done; disabled");
        drivetrainSubsystem.driveVolts(0.0);
        logger.sendData();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}

