package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
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

    private PIDController radialTranslationController;
    private PIDController lateralTranslationController;
    private PIDController rotationalController;
    private DrivetrainSubsystem drivetrainSubsystem;
    private Collector collectorSubsystem;
    private VisionBox visionBox;
    private Pose2d ballPose;
    private boolean initializationFailed;
    
    public VisionBoxCollectBallCommand(VisionBox visionBox, DrivetrainSubsystem drivetrain, Collector collector) {

        radialTranslationController = new PIDController(VisionBoxConstants.RAIDAL_KP, VisionBoxConstants.RAIDAL_KI, VisionBoxConstants.RAIDAL_KD); // controls radial movement (to and from the ball)
        lateralTranslationController = new PIDController(VisionBoxConstants.LATERAL_KP, VisionBoxConstants.LATERAL_KI, VisionBoxConstants.LATERAL_KD); // controls lateral movement (left/right from robot perspective) 
        rotationalController = new PIDController(VisionBoxConstants.ROTATIONAL_KP, VisionBoxConstants.ROTATIONAL_KI, 0); // controls rotational movement (about robot center)

        drivetrainSubsystem = drivetrain;
        collectorSubsystem = collector;
        this.visionBox = visionBox;

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
        radialTranslationController.reset();
        lateralTranslationController.reset();
        rotationalController.reset();

        //disable field relative drive
        drivetrainSubsystem.disableFieldRelative();        

        // get first ball pose
        Transform2d ballTransform = visionBox.getFirstBallTransform2d();
        initializationFailed = ballTransform == null; // if the the first ball doesn't exist, fail starting the command

        // get the first ball Pose2d relative to the field
        ballPose = drivetrainSubsystem.getPose().plus(ballTransform);
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
        double robotToBallAngle = Math.atan2(robotToBallTranslation.getY(), robotToBallTranslation.getX()); //the angle from the robot to the ball relative to the robot


        if (this.isAimed()) {
            //the robot is aimed, so the ideal location is the furthest point from the robot on a circle OVERSHOOT_DISTANCE_METERS from the ball
            Transform2d ballToIdealPoseTransform = new Transform2d(new Translation2d(VisionBoxConstants.OVERSHOOT_DISTANCE_METERS, new Rotation2d(robotToBallAngle)), new Rotation2d(0));
            idealPose = ballPose.plus(ballToIdealPoseTransform);
        } else {
            //the robot isn't aimed, so the ideal location is the nearest point along a circle MINIMUM_UNAIMED_DISTANCE_METERS from the ball
            //using the angle from the robot to the ball, create a new transform from the ball position translated -MINIMUM_UNAIMED_DISTANCE_METERS meters backwards towards the robot
            Transform2d ballToIdealPoseTransform = new Transform2d(new Translation2d(-VisionBoxConstants.MINIMUM_UNAIMED_DISTANCE_METERS, new Rotation2d(robotToBallAngle)), new Rotation2d(0));
            idealPose = ballPose.plus(ballToIdealPoseTransform);
        }

        //find translation between the robot and the ideal pose relative to the robot
        Transform2d distance = new Transform2d(drivetrainSubsystem.getPose(), idealPose);

        //calculate PIDs
        double radialOutput = radialTranslationController.calculate(distance.getY());
        double lateralOutput = lateralTranslationController.calculate(distance.getX());
        double rotationalOutput = rotationalController.calculate(Math.atan2(distance.getX(),distance.getY())); //atan2 treats positive x axis as 0 degreees, we want positive y axis (aimed directly at the ball) to be 0

        //drive the robot
        drivetrainSubsystem.aim(lateralOutput, radialOutput, rotationalOutput);
        //re-query the ball pose 
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
        drivetrainSubsystem.enableFieldRelative();
        // TODO: do this

    }

    /**
     * This method is invoked at the end of each Command Scheduler iteration. It returns true when
     * the drivetrain is aimed, the flywheel is at the specified speed, and the robot is within
     * optimal shooting distance.
     */

    public boolean isAimed() {
        return visionBox.getFirstBallTx() < Math.toRadians(VisionBoxConstants.AIM_TOLERANCE_DEGREES);
    }
    @Override
    public boolean isFinished() {

        // TODO: do this
        return initializationFailed || false;
    }

}
