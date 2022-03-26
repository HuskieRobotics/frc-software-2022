package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

/**
 * 
 *
 */
public class FollowPath extends PPSwerveControllerCommand {
    private ProfiledPIDController thetaController;
    private DrivetrainSubsystem drivetrainSubsystem;
    private PathPlannerTrajectory trajectory;
    private boolean initialPath;

    public FollowPath(PathPlannerTrajectory trajectory, ProfiledPIDController thetaController, DrivetrainSubsystem subsystem, boolean initialPath) {
        super(trajectory, subsystem::getPose, subsystem.getKinematics(), new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                subsystem::setSwerveModuleStates, subsystem);

        this.thetaController = thetaController;
        this.drivetrainSubsystem = subsystem;
        this.trajectory = trajectory;
        this.initialPath = initialPath;
    }

    @Override
    public void initialize() {
        super.initialize();

        this.drivetrainSubsystem.enableStackTraceLogging(false);

        if(initialPath) {
            this.drivetrainSubsystem.setGyroOffset(this.trajectory.getInitialState().holonomicRotation.getDegrees());

            // Reset odometry to the starting pose of the trajectory.
            this.drivetrainSubsystem.resetOdometry(this.trajectory.getInitialState());
        }

        // reset the theta controller such that old accumuldated ID values aren't used with the new path
        //      this doesn't matter if only the P value is non-zero, which is the current behavior
        this.thetaController.reset(this.drivetrainSubsystem.getPose().getRotation().getRadians());

        

    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrainSubsystem.enableStackTraceLogging(true);
        this.drivetrainSubsystem.stop();
        super.end(interrupted);
    }
}