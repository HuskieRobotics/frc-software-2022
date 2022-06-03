package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

/**
 * This command, when executed, instructs the drivetrain subsystem to follow the specified
 * trajectory, persumably during the autonomous period. The superclass' execute method invokes
 * the drivetrain subsystem's setSwerveModuleStates method to follow the trajectory.
 * 
 * Requires: the Drivetrain subsystem (handled by superclass)
 * Finished When: the time of the specified path has elapsed (handled by superclass)
 * At End: stops the drivetrain
 */
public class FollowPath extends PPSwerveControllerCommand {
    private ProfiledPIDController thetaController;
    private DrivetrainSubsystem drivetrainSubsystem;
    private PathPlannerTrajectory trajectory;
    private boolean initialPath;

    /**
     * Constructs a new FollowPath object.
     * 
     * @param trajectory the specified trajectory created by PathPlanner
     * @param thetaController the PID controller for the drivetrain's rotation
     * @param subsystem the drivetrain subsystem required by this command
     * @param initialPath true, if this trajectory is the first in a sequence of trajectories
     *      or the only trajector, in which case the gyro and odometry will be initialized to
     *      match the start of trajectory; false, if this trajectory is a subsequent trajectory
     *      in which case the gyro and odometry will not be re-initialized in order to ensure a
     *      smooth transition between trajectories
     */
    public FollowPath(PathPlannerTrajectory trajectory, ProfiledPIDController thetaController, DrivetrainSubsystem subsystem, boolean initialPath) {
        super(trajectory, subsystem::getPose, subsystem.getKinematics(), new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                subsystem::setSwerveModuleStates, subsystem);

        this.thetaController = thetaController;
        this.drivetrainSubsystem = subsystem;
        this.trajectory = trajectory;
        this.initialPath = initialPath;
    }

    /**
     * This method is invoked once when this command is scheduled. If the trajectory is the first
     * in a sequence of trajectories or the only trajector, initialize the gyro and odometry to
     * match the start of trajectory. PathPlanner sets the origin of the field to the lower left
     * corner (i.e., the corner of the field to the driver's right). Zero degrees is away from the
     * driver and increases in the CCW direction. It is critical that this initialization occurs in
     * this method and not the constructor as this object is constructed well before the command is
     * scheduled.
     */
    @Override
    public void initialize() {
        super.initialize();

        this.drivetrainSubsystem.enableStackTraceLogging(false);

        if(initialPath) {

            // incorporate this line into the resetOdometry method
            this.drivetrainSubsystem.setGyroOffset(this.trajectory.getInitialState().holonomicRotation.getDegrees());

            // Reset odometry to the starting pose of the trajectory.
            this.drivetrainSubsystem.resetOdometry(this.trajectory.getInitialState());
        }

        // reset the theta controller such that old accumuldated ID values aren't used with the new path
        //      this doesn't matter if only the P value is non-zero, which is the current behavior
        this.thetaController.reset(this.drivetrainSubsystem.getPose().getRotation().getRadians());

        

    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the
     * motion of the drivetrain.
     * 
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
        this.drivetrainSubsystem.enableStackTraceLogging(true);
        this.drivetrainSubsystem.stop();
        super.end(interrupted);
    }
}