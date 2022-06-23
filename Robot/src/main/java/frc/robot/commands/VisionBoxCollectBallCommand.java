package frc.robot.commands;

import frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.VisionBox;

/**
 * This command, when executed, instructs the drivetrain subsystem to move based on the vision box's 
 * outputs. It utilises 3 PID controllers, two for vertical and horizontal translation and one for 
 * rotational, to align the robot to a cargo and collect it.
 * 
 * Requires: the drivetrain and collector subsystems
 * Finished When: the robot intakes a cargo
 * At End: stops the drivetrain and retracts the intake
 */
public class VisionBoxCollectBallCommand extends CommandBase {

    private ProfiledPIDController xTranslationController;
    private ProfiledPIDController yTranslationController;
    private ProfiledPIDController rotationalController;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Collector collectorSubsystem;
    private Storage storageSubsystem;
    private VisionBox visionBoxSubsystem;
    private Pose2d ballPose;
    private boolean initializationFailed;
    private int ticksWithoutBall;
    private int startingCargoCount;
    private boolean initialFieldRelativeState;
    
    public VisionBoxCollectBallCommand(VisionBox visionBox, DrivetrainSubsystem drivetrain, Collector collector, Storage storage) {
        //X,Y and angle are all in robot-relative coordinate system (x is forwards and backwards, y is side to side, +angle is counterclockwise)
        xTranslationController = new ProfiledPIDController(VisionBoxConstants.X_KP, VisionBoxConstants.X_KI, VisionBoxConstants.X_KD, new TrapezoidProfile.Constraints(5, 10));
        yTranslationController = new ProfiledPIDController(VisionBoxConstants.Y_KP, VisionBoxConstants.Y_KI, VisionBoxConstants.Y_KD, new TrapezoidProfile.Constraints(5, 10)); 
        rotationalController = new ProfiledPIDController(VisionBoxConstants.ROTATIONAL_KP, VisionBoxConstants.ROTATIONAL_KI, 0, new TrapezoidProfile.Constraints(5, 10)); 

        drivetrainSubsystem = drivetrain;
        collectorSubsystem = collector;
        storageSubsystem = storage;
        visionBoxSubsystem = visionBox;

        addRequirements(drivetrainSubsystem);
        addRequirements(collectorSubsystem);
    }
    
    /**
     * This method is invoked once when this command is scheduled. It resets the PID controller
     * for the drivetrain rotation. It is critical that this initialization occurs in this method
     * and not the constructor as this command is constructed once when the RobotContainer is
     * created, but this method is invoked each time this command is scheduled.
     */
    @Override
    public void initialize() {
        // critical to reset the PID controllers each time this command is initialized to reset any accumulated values due to non-zero I or D values
        xTranslationController.reset(new TrapezoidProfile.State(0, drivetrainSubsystem.getVelocityX()));
        yTranslationController.reset(new TrapezoidProfile.State(0, drivetrainSubsystem.getVelocityY()));
        rotationalController.reset(new TrapezoidProfile.State(0, drivetrainSubsystem.getVelocityRotational()));

        //reset ticksWithoutBall
        ticksWithoutBall = 0;
        
        //disable field relative drive
        initialFieldRelativeState = drivetrainSubsystem.getFieldRelative();
        drivetrainSubsystem.disableFieldRelative();        

        // get first ball pose
        Transform2d ballTransform = visionBoxSubsystem.getFirstBallTransform2d();
        initializationFailed = ballTransform == null; // if the the first ball doesn't exist, fail starting the command

        // get the first ball Pose2d relative to the field
        ballPose = drivetrainSubsystem.getPose().plus(ballTransform);

        //find the number of balls in the storage
        startingCargoCount = storageSubsystem.getNumberOfCargoInStorage();
    }

    /**
     * This method will be invoked every iteration of the Command Scheduler. It repeatedly
     * instructs the drivetrain subsytem to move to the ideal location. Notably, it doesn't rely on visionBox to continue funcitoning, as a ball may blink out of view if it is covered by the intake
     */
    @Override
    public void execute() {
        //calculate the ideal location to PID to, either the ball if the robot is aimed at it, or MINIMUM_UNAIMED_DISTANCE_METERS meters from the ball if it isn't.
        Pose2d idealPose;

        //get the angle from the robot to the ball
        Translation2d robotToBallTranslation = (new Transform2d(drivetrainSubsystem.getPose(), ballPose)).getTranslation(); //the translation from the robot to the ball
        double robotToBallAngle = Math.atan2(robotToBallTranslation.getY(), robotToBallTranslation.getX()); //TODO: verify this math
        if (this.isAimed()) {
            //the robot is aimed, so the ideal location is the furthest point from the robot on a circle OVERSHOOT_DISTANCE_METERS from the ball
            Transform2d ballToIdealPoseTransform = new Transform2d(new Translation2d(VisionBoxConstants.OVERSHOOT_DISTANCE_METERS, new Rotation2d(robotToBallAngle)), new Rotation2d(0));
            idealPose = ballPose.plus(ballToIdealPoseTransform);

            //deploy the collector once we're aimed to the target
            collectorSubsystem.enableCollector();
        } else {
            //the robot isn't aimed, so the ideal location is the nearest point along a circle MINIMUM_UNAIMED_DISTANCE_METERS from the ball
            //using the angle from the robot to the ball, create a new transform from the ball position translated -MINIMUM_UNAIMED_DISTANCE_METERS meters backwards towards the robot
            Transform2d ballToIdealPoseTransform = new Transform2d(new Translation2d(-VisionBoxConstants.MINIMUM_UNAIMED_DISTANCE_METERS, new Rotation2d(robotToBallAngle)), new Rotation2d(0));
            idealPose = ballPose.plus(ballToIdealPoseTransform);
        }

        //find translation between the robot and the ideal pose relative to the robot
        Transform2d distance = new Transform2d(drivetrainSubsystem.getPose(), idealPose);

        //calculate PIDs
        double xOutput = xTranslationController.calculate(distance.getX(), new TrapezoidProfile.State(0, VisionBoxConstants.Y_MAX_VELOCITY)); //end moving at max velocity in the x direction
        double yOutput = yTranslationController.calculate(distance.getY(), new TrapezoidProfile.State(0, 0)); //end without strafing
        double rotationalOutput = rotationalController.calculate(robotToBallAngle, new TrapezoidProfile.State(0, 0)); //end without rotating

        // UNCOMMENT THIS IF THE ROBOT IS MOVING TOO QUICKLY WHEN "AIMING"
        // final double UNAIMED_MOVEMENT_MULTIPLIER = .66;
        // if (!isAimed()) { //if the robot isn't aimed, make it move really slowly as we don't trust horzontial angles too far from 0 deg
        //     xOutput *= UNAIMED_MOVEMENT_MULTIPLIER;
        //     yOutput *= UNAIMED_MOVEMENT_MULTIPLIER;
        // }
        
        //drive the robot
        drivetrainSubsystem.aim(xOutput, yOutput, rotationalOutput);
    
        //update the ball pose
        Transform2d ballTransform = visionBoxSubsystem.getFirstBallTransform2d();

        if (ballTransform != null) { //if a ball is found, check if it's the same ball
            Pose2d newPose = drivetrainSubsystem.getPose().plus(ballTransform);
            //find the distance between the new and old ball pose
            double ballDisplacement = (new Transform2d(ballPose, newPose)).getTranslation().getNorm();

            if (ballDisplacement < ticksWithoutBall * VisionBoxConstants.MAX_DISPLACEMENT_PER_TICK_METERS) { //ensure the ball didn't move more than acceptable
                ballPose = newPose;
                ticksWithoutBall = 0;
            } else {
                ticksWithoutBall++;
            }

        } else {
            ticksWithoutBall++;
        }
    }

    /**
     * This method will be invoked when this command finishes or is interrupted. It stops the
     * motion of the drivetrain. It does not stop the motion of the flywheel.
     * 
     * @param interrupted true if the command was interrupted by another command being scheduled
     */
    @Override
    public void end(boolean interrupted) {
        //reenable field relative drive
        if (initialFieldRelativeState) {
            drivetrainSubsystem.enableFieldRelative();
        }
        collectorSubsystem.disableCollector();
        drivetrainSubsystem.stop();
        // if (storageSubsystem.getNumberOfCargoInStorage() == 2) { //bring in the collector
        //     collectorSubsystem.disableCollector();
        // }

    }

    public boolean isAimed() { //dont use visionbox for isaimed, instead use stored pose (as that is what we are tracking)
        Translation2d robotToBallTranslation = (new Transform2d(drivetrainSubsystem.getPose(), ballPose)).getTranslation(); //the translation from the robot to the ball
        
        return robotToBallTranslation.getY() < VisionBoxConstants.AIM_TOLERANCE_METERS; //translational error approach (require to be within a certain horizontal distance)

        // double robotToBallAngle = Math.atan2(robotToBallTranslation.getY(), robotToBallTranslation.getX()); //rotational error approach (require to be within a certain degree range)
        // return robotToBallAngle < Math.toRadians(VisionBoxConstants.AIM_TOLERANCE_DEGREES);
    }
    
    @Override
    public boolean isFinished() {
        return initializationFailed || 
        storageSubsystem.getNumberOfCargoInStorage() > startingCargoCount ||
        storageSubsystem.getNumberOfCargoInStorage() == 2;
    }

}
