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

    public FollowPath(PathPlannerTrajectory trajectory, ProfiledPIDController thetaController, DrivetrainSubsystem subsystem) {
        super(trajectory, subsystem::getPose, subsystem.getKinematics(), new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                subsystem::setSwerveModuleStates, subsystem);

        this.thetaController = thetaController;
        this.drivetrainSubsystem = subsystem;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        super.initialize();

        // reset the theta controller such that old accumuldated ID values aren't used with the new path
        //      this doesn't matter if only the P value is non-zero, which is the current behavior
        this.thetaController.reset(this.drivetrainSubsystem.getPose().getRotation().getRadians());

        // Reset odometry to the starting pose of the trajectory.
        this.drivetrainSubsystem.resetOdometry(this.trajectory.getInitialState());

    }

    // @Override
    // public void execute(){

    // }
}