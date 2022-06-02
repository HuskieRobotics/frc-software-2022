package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A single instance of this command should be created in the RobotContainer class and initialized
 * with the joystick inputs as the suppliers. As a default command, this command will be scheduled
 * whenever no other commands that require the drivetrain subsystem are scheduled.
 * 
 * Requires: the Drivetrain subsystem.
 * Finished When: never unless interrupted
 * At End: stops the drivetrain
 */
public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /**
     * Construct a new DefaultDriveCommand object.
     * 
     * @param drivetrainSubsystem the Drivetrain subsystem required by this command
     * @param translationXSupplier supplies the desired velocity in the x direction (m/s)
     * @param translationYSupplier supplies the desired velocity in the y direction (m/s)
     * @param rotationSupplier supplies the desried rotational velocity (m/s)
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
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

    }

    /**
     * This method will be invoked every iteration of the Command Scheduler. This method
     * should only invoke the drive method on the Drivetrain subsystem. Any other logic needs to
     * be in the drive method as this method is only one of several methods that invoke the drive
     * method.
     */
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        m_drivetrainSubsystem.drive(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble());
        // m_drivetrainSubsystem.drive(new
        // ChassisSpeeds(m_translationXSupplier.getAsDouble(),
        // m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble()));
    }

    /**
     * This method will be invoked when this command is interrupted. The stop method of the
     * Drivetrain subsystem needs to be invoked or else the robot will continue to move.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stop();
    }
}
