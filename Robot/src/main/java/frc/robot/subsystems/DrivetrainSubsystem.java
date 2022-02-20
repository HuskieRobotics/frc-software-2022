// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 13.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
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
                        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
                        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        private static final double SWERVE_PID_D = 0;

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        // FIXME Remove if you are using a Pigeon
        private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;
        private boolean isFieldRelative;
        private boolean isJoystickControlAllowed;
        private final Translation2d m_frontLeftLocation;
        private final Translation2d m_frontRightLocation;
        private final Translation2d m_backLeftLocation;
        private final Translation2d m_backRightLocation;
        private Translation2d m_robotCenter;

        private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                        Rotation2d.fromDegrees(m_pigeon.getYaw()));

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SimpleMotorFeedforward feedForward;

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                this.isFieldRelative = true;
                this.isJoystickControlAllowed = true;
                this.m_robotCenter = new Translation2d(0, 0);
                this.m_frontLeftLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                DRIVETRAIN_WHEELBASE_METERS / 2.0);
                this.m_frontRightLocation = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                -DRIVETRAIN_WHEELBASE_METERS / 2.0);
                this.m_backLeftLocation = new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                DRIVETRAIN_WHEELBASE_METERS / 2.0);
                this.m_backRightLocation = new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                -DRIVETRAIN_WHEELBASE_METERS / 2.0);

                m_pigeon.setYaw(0.0);
                m_pigeon.setFusedHeading(0.0);

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
                // FIXME Setup motor configuration
                m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk3SwerveModuleHelper.GearRatio.FAST,
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
                m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk3SwerveModuleHelper.GearRatio.FAST,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk3SwerveModuleHelper.GearRatio.FAST,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk3SwerveModuleHelper.GearRatio.FAST,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);


                this.feedForward = new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter);

                tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
                tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
                tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());
                tab.addNumber("Pose Rotation", () -> m_odometry.getPoseMeters().getRotation().getDegrees());
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                // FIXME Remove if you are using a Pigeon
                m_pigeon.setFusedHeading(0.0);

        }

        public void zeroPoseGyroscope() {
                m_odometry.resetPosition(
                                new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                                Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
        }

        public Rotation2d getGyroscopeRotation() {
                // FIXME Remove if you are using a Pigeon
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose)
        {
                m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_pigeon.getYaw()));
        }
        
        public void drive(ChassisSpeeds chassisSpeeds) {
                if (isJoystickControlAllowed) {
                        m_chassisSpeeds = chassisSpeeds;
                }

        }

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

                /*
                 * if(this.getFieldRelative()){
                 * m_chassisSpeeds =
                 * ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds.vxMetersPerSecond,
                 * m_chassisSpeeds.vyMetersPerSecond, m_chassisSpeeds.omegaRadiansPerSecond,
                 * getGyroscopeRotation());
                 * }
                 */

                 // FIXME: this should only occur in teleop
                // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                // m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                //                 states[0].angle.getRadians());
                // m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                //                 states[1].angle.getRadians());
                // m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                //                 states[2].angle.getRadians());
                // m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                //                 states[3].angle.getRadians());
        }
        public void setSwerveModuleStates(SwerveModuleState[] states) {
                m_frontLeftModule.set(this.calculateFeedforwardVoltage(states[0].speedMetersPerSecond), states[0].angle.getRadians());
                m_frontRightModule.set(this.calculateFeedforwardVoltage(states[1].speedMetersPerSecond), states[0].angle.getRadians());
                m_backLeftModule.set(this.calculateFeedforwardVoltage(states[2].speedMetersPerSecond), states[0].angle.getRadians());
                m_backRightModule.set(this.calculateFeedforwardVoltage(states[3].speedMetersPerSecond), states[0].angle.getRadians());
        }

        private double calculateFeedforwardVoltage(double velocity) {
                double voltage = this.feedForward.calculate(velocity);
                if(voltage > MAX_VOLTAGE) {
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

        public void fieldRelativeToggle() {
                if (isFieldRelative) {
                        this.isFieldRelative = false;
                } else {
                        this.isFieldRelative = true;
                }
        }

        public void feildRelativeOn() {
                this.isFieldRelative = true;
        }

        public void feildRelativeOff() {
                this.isFieldRelative = false;
        }

        public void enableJoystickControlls() {
                this.isJoystickControlAllowed = true;
        }

        public void disableJoystickControlls() {
                this.isJoystickControlAllowed = false;
        }

        public void setXStance() {
                // FL
                m_frontLeftModule.set(0, Math.PI * 5 / 4);
                // FR
                m_frontRightModule.set(0, Math.PI * -5 / 4);
                // BL
                m_backLeftModule.set(0, Math.PI * 3 / 4);
                // BR
                m_backRightModule.set(0, Math.PI * -3 / 4);
        }

        public void setCenterGrav(Translation2d center) {
                this.m_robotCenter = center;

        }

        public void zeroGyroscopePose() {

        }
}
