package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * 
 *
 */
public class FollowPath extends PPSwerveControllerCommand {

    public FollowPath(PathPlannerTrajectory trajectory, DrivetrainSubsystem subsystem) {
        super(trajectory, subsystem::getPose, subsystem.getKinematics(), new PIDController(2.2956, 0, 0),
                new PIDController(2.2956, 0, 0), new ProfiledPIDController(0, 0, 0, new Constraints(1, 1)),
                subsystem::setSwerveModuleStates, subsystem);
        addRequirements(subsystem);
    }
}