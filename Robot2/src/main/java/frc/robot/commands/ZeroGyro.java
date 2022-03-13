package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;

    
/**
 *
 */
public class ZeroGyro extends CommandBase {

        private final DrivetrainSubsystem m_drivetrainSubsystem;
 

    public ZeroGyro(DrivetrainSubsystem subsystem) {

        m_drivetrainSubsystem = subsystem;
        addRequirements(m_drivetrainSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //m_drivetrainSubsystem.zeroGyroscope();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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
