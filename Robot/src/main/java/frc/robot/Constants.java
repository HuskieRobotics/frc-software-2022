// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
  
  public static final boolean TUNING = false; 

  /* DRIVETRAIN CONSTANTS */

  public static final class DrivetrainConstants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 16;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 15;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 17;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(149.589843);
    // 329.58984375
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(172.96875);
    // 352.96875
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(103.27148);
    // 283.271484375
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(117.333984);
    //297.333984375

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.5969;// 23.5 inches

    public static final int PIGEON_ID = 18;
  }

  public static final class CollectorConstants{
      public static final double OUTTAKE_POWER = -0.7;//FIX_ME
      public static final int COLLECTOR_MOTOR_ID = 5;
      public static final int PEUNAMATICS_HUB_CAN_ID = 20; 
      public static final int COLLECTOR_SOLENOID_CHANNEL = 1;
      public static final double COLLECTOR_DEFUALT_SPEED = 0.7; //FIX_ME change this to desired speed

  }
  public static final class StorageConstants{
      public static final double OUTTAKE_POWER = -.4;//FIX_ME
      public static final int SHOOTER_SENSOR = 1;
      public static final int COLLECTOR_SENSOR = 0;
      public static final int STORAGE_MOTOR_ID = 4;
      public static final double STORAGE_DEFAULT_SPEED = 0.4; //FIX_ME change this to desired speed
  }

    public static final class AutoConstants {

      // from sysId tool
      public static final double ksVolts = 0.55493;
      public static final double kvVoltSecondsPerMeter = 2.3014;
      public static final double kaVoltSecondsSquaredPerMeter = 0.12872;

      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 2.2956;    // from sysId tool
      public static final double kPYController = 2.2956;    // from sysId tool
      public static final double kPThetaController = 4.9;     // tune after verifying non-holonomic motion
  
      // Constraint for the motion profilied robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
