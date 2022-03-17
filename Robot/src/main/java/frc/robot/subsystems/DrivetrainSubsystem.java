// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LimelightAlignToTargetCommand;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.Map;

public class DrivetrainSubsystem extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 13.0;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

        private Translation2d centerGravity = new Translation2d();        // default to (0,0)
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        private final Pigeon2 m_pigeon = new Pigeon2(PIGEON_ID);

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;
        private boolean isFieldRelative;
        private boolean isXstance;
        // private Translation2d m_robotCenter;
        private NetworkTableEntry fieldRelativeNT;

        private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                        Rotation2d.fromDegrees(m_pigeon.getYaw()));

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SimpleMotorFeedforward feedForward;

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                this.isFieldRelative = false;
                this.isXstance = false;
                // this.m_robotCenter = new Translation2d(0,0);

                m_pigeon.setYaw(0.0);

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // Mk3SwerveModuleHelper.createFalcon500(...)
                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.

                m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                // This is the ID of the drive motor
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);

                this.feedForward = new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter);

                tab.add("drivetrain", this);
                tab.addBoolean("Launchpad Dist", () -> isAtLaunchpadDistance());
                tab.addBoolean("Wall Dist", () -> isAtWallDistance());
                tab.addBoolean("Is Aimed", () -> isAimed());
                tab.addNumber("Limelight Dist", () -> getLimelightDistanceIn());
                tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
                tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
                tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());
                tab.addNumber("Pose Rotation", () -> m_odometry.getPoseMeters().getRotation().getDegrees());
                this.fieldRelativeNT = Shuffleboard.getTab("Drivetrain")
                                .add("FieldRelativeState", this.isFieldRelative)
                                .getEntry();
                tab.addBoolean("isXstance", this :: isXstance);
                tab.add("Enable XStance", new InstantCommand(() -> this.enableXstance()));
                tab.add("Disable XStance", new InstantCommand(() -> this.disableXstance()));
                tab.addNumber("CoG X", () -> this.centerGravity.getX());
                tab.addNumber("CoG Y", () -> this.centerGravity.getY());
                tab.add("align to target", new LimelightAlignToTargetCommand(this));

                if (TUNING) {
                        // Add indicators and controls to this Shuffleboard tab to assist with
                        // interactively tuning the system.
            
                        Shuffleboard.getTab("Limelight")
                                .add("Angle Tolerance", LIMELIGHT_ALIGNMENT_TOLERANCE)
                                .withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", 0, "max", 10)) // specify widget properties here
                                .getEntry()
                                .addListener(event -> {
                                        LIMELIGHT_ALIGNMENT_TOLERANCE = event.getEntry().getValue().getDouble();
                                }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            
                        Shuffleboard.getTab("Limelight")
                                .add("Limelight P", LIMELIGHT_P)
                                .withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", -2.0, "max", 2.0)) // specify widget properties here
                                .getEntry()
                                .addListener(event -> {
                                        LIMELIGHT_P = event.getEntry().getValue().getDouble();
                                }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
                }



        }

        /*
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */

        public void zeroGyroscope() {
                m_pigeon.setYaw(0.0);
        }

        public void zeroPoseGyroscope() {
                m_odometry.resetPosition(
                                new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                                Rotation2d.fromDegrees(m_pigeon.getYaw()));
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void resetOdometry(PathPlannerState state) {
                m_odometry.resetPosition(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation),
                 Rotation2d.fromDegrees(m_pigeon.getYaw()));
        }

        // Implement change in center of gravity here
        public void drive(double translationXSupplier, double translationYSupplier, double rotationSupplier) {
        if (isXstance) {
                this.setXStance();
        }
        else{
                if (isFieldRelative) {
                        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translationXSupplier,
                                        translationYSupplier,
                                        rotationSupplier,
                                        getGyroscopeRotation());

                } else {
                        m_chassisSpeeds = new ChassisSpeeds(
                                        translationXSupplier,
                                        translationYSupplier,
                                        rotationSupplier);
                }
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds, centerGravity);
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }}

        @Override
        public void periodic() {
                m_odometry.update(Rotation2d.fromDegrees(m_pigeon.getYaw()),
                                new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
                                                new Rotation2d(m_frontLeftModule.getSteerAngle())),
                                new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
                                                new Rotation2d(m_frontRightModule.getSteerAngle())),
                                new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
                                                new Rotation2d(m_backLeftModule.getSteerAngle())),
                                new SwerveModuleState(m_backRightModule.getDriveVelocity(),
                                                new Rotation2d(m_backRightModule.getSteerAngle())));
        }

        public void setSwerveModuleStates(SwerveModuleState[] states) {
                m_frontLeftModule.set(this.calculateFeedforwardVoltage(states[0].speedMetersPerSecond),
                                states[0].angle.getRadians());
                m_frontRightModule.set(this.calculateFeedforwardVoltage(states[1].speedMetersPerSecond),
                                states[1].angle.getRadians());
                m_backLeftModule.set(this.calculateFeedforwardVoltage(states[2].speedMetersPerSecond),
                                states[2].angle.getRadians());
                m_backRightModule.set(this.calculateFeedforwardVoltage(states[3].speedMetersPerSecond),
                                states[3].angle.getRadians());
        }

        private double calculateFeedforwardVoltage(double velocity) {
                double voltage = this.feedForward.calculate(velocity);
                if (voltage > MAX_VOLTAGE) {
                        return MAX_VOLTAGE;
                }
                return voltage;
        }

        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        public boolean getFieldRelative() {
                return isFieldRelative;
        }

        public boolean enableFieldRelative() {
                this.isFieldRelative = true;
                this.fieldRelativeNT.setBoolean(this.isFieldRelative);
                return this.isFieldRelative;
        }

        public boolean disableFieldRelative() {
                this.isFieldRelative = false;
                this.fieldRelativeNT.setBoolean(this.isFieldRelative);
                return this.isFieldRelative;
        }

        public void setXStance() {
                // FL
                m_frontLeftModule.set(0, (Math.PI/2 - Math.atan(22.5 / 23.5)));
                // FR
                m_frontRightModule.set(0, (Math.PI/2 + Math.atan(22.5 / 23.5)));
                // BL
                m_backLeftModule.set(0, (Math.PI/2 + Math.atan(22.5 / 23.5)));
                // BR
                m_backRightModule.set(0, (3.0/2.0 * Math.PI - Math.atan(22.5 / 23.5)));
        }

        /*
         * Make center of gravity into a instance command and have the instance command
         * change based on the method
         * need to implement the instance variable within the periodic method
         * Create either a whenPressed or toggleWhenPressed method in robotContainer
         * Use below method to test
         */
        public void setCenterGrav(double x, double y) {
                this.centerGravity = new Translation2d(x, y);
        }

        public void resetCenterGrav() {
                setCenterGrav(0.0, 0.0);
        }

        public double getLimelightX(){
                return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
           }

        public double getLimelightDistanceIn() {
                double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

                double d = (LimelightConstants.HUB_H - LimelightConstants.ROBOT_H)
                        / (Math.tan(Math.toRadians(LimelightConstants.LIMELIGHT_MOUNT_ANGLE * +LimelightConstants.LIMELIGHT_ANGLE_OFFSET + ty)));

                return d;
        }

        public boolean isAtLaunchpadDistance() {
                double dist = getLimelightDistanceIn();
                return Math.abs(LimelightConstants.HUB_LAUNCHPAD_DISTANCE - dist) <= LimelightConstants.DISTANCE_TOLERANCE;
        }

        public boolean isAtWallDistance() {
                double dist = getLimelightDistanceIn();
                return Math.abs(LimelightConstants.HUB_WALL_DISTANCE - dist) <= LimelightConstants.DISTANCE_TOLERANCE;
        }

        public void aim(double translationXSupplier, double translationYSupplier, double rotationSupplier) {
                if (rotationSupplier > 0) {     // FIXME: verify this is clockwise
                        setCenterGrav(DrivetrainConstants.ROBOT_LENGTH_WITH_BUMPERS/2,
                                -DrivetrainConstants.ROBOT_WIDTH_WITH_BUMPERS/2);
                }
                else {  // counterclockwise
                        setCenterGrav(DrivetrainConstants.ROBOT_LENGTH_WITH_BUMPERS/2,
                                DrivetrainConstants.ROBOT_WIDTH_WITH_BUMPERS/2);
                }

                drive(translationXSupplier, translationYSupplier, rotationSupplier);

        }

        public boolean isAimed() {
                return Math.abs(0.0 - getLimelightX()) - LIMELIGHT_ALIGNMENT_TOLERANCE <= 0;
        }

        public void enableXstance() {
                this.isXstance = true;
        }
        public void disableXstance() {
                this.isXstance = false;
        }

        public boolean isXstance() {
                return isXstance;
        }

           

}
