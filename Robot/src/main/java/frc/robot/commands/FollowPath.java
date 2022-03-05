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
    public FollowPath(PathPlannerTrajectory trajectory, ProfiledPIDController thetaController, DrivetrainSubsystem subsystem) {
        super(trajectory, subsystem::getPose, subsystem.getKinematics(), new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                subsystem::setSwerveModuleStates, subsystem);
        addRequirements(subsystem);

        // Reset odometry to the starting pose of the trajectory.
        subsystem.resetOdometry(trajectory.getInitialState());

    }

    // @Override
    // public void execute(){

    // }
}