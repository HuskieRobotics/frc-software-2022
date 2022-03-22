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
  public static final boolean COMMAND_LOGGING = false;

  /* DRIVETRAIN CONSTANTS */

  public static final class DrivetrainConstants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;//16;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; //15;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;//17;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(118.0371);
    // 149.589843
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;//10;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;//9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;//11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(103.2715);
    // 172.96875
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;//13;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;//12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER =11;// 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(172.7051);
    // 103.27148
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;//7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;//6;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;//8;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(149.5020);
    // 117.333984

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

    public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89;
    public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91;
    public static final double EVASIVE_ROTATION_COG_SHIFT_MAGNITUDE = 0.707;  // a bit beyond the bumper permimeter
    public static final double COG_OFFSET = 45;

    public static final int PIGEON_ID = 18;
    public static final int TIMEOUT_MS = 30;

    /* Limelight */
    public static  double LIMELIGHT_P = 0.055; // FIXME: adjust when tuning and then make final
    public static  double LIMELIGHT_ALIGNMENT_TOLERANCE = 1; // degrees; FIXME: adjust when testing and then make final
  }

  public static final class CollectorConstants {
    public static final double OUTTAKE_POWER = -0.7;// FIX_ME
    public static final int COLLECTOR_MOTOR_ID = 5;
    public static final int PEUNAMATICS_HUB_CAN_ID = 20;
    public static final int COLLECTOR_SOLENOID_CHANNEL = 0;
    public static final double COLLECTOR_DEFUALT_SPEED = 0.9; // FIX_ME change this to desired speed; was 0.7
    public static final int TIMEOUT_MS = 30;

  }

  public static final class AutoConstants {

    // from sysId tool
    public static final double ksVolts = 0.55493;
    public static final double kvVoltSecondsPerMeter = 2.3014;
    public static final double kaVoltSecondsSquaredPerMeter = 0.12872;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/3;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/3;

    public static final double kPXController = 2.2956; // from sysId tool
    public static final double kPYController = 2.2956; // from sysId tool
    public static final double kPThetaController = 4.9; // tune after verifying non-holonomic motion

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class FlywheelConstants {
    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int VELOCITY_TOLERANCE = 300; 
    public static final int LEFT_FLYWHEELMOTOR_CANID = 1;
    public static final int RIGHT_FLYWHEELMOTOR_CANID = 2;
    public final static Gains GAINS_VELOCITY = new Gains(0.5 /* kP */, 0 /* kI */, 0 /* kD */, .0438 /* kF */,
        0 /* kIzone */, 1.00 /* kPeakOutput */);
    public final static int MAX_FLYWHEEL_VELOCITY = 17500;
    public static final int WALL_SHOT_VELOCITY = 7782; // FIXME tune this
    public static final int FENDER_SHOT_VELOCITY = 7799; // FIXME tune this
    public static final int LAUNCH_PAD_VELOCITY = 8447; // FIXME tune this
    public static final int SHOOT_SLOW_VELOCITY = 4000; // FIXME tune this
    public static final int SHOT_VELOCITY_DIP = 500;  // FIXME: tune this
    public static final double REVERSE_POWER = -0.2; // FIXME: tune this


  }

  public static class HoodConstants {
    public static final int HOOD_MOTOR_ID = 3;
    public static final double KP = 0; // FIXME find hood pid p value
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KIz = 0;
    public static final double KFF = 0;
    public static final double K_MAX_OUTPUT = 1;
    public static final double K_MIN_OUTPUT = -1;
    public static final int TIMEOUT_MS = 30;

    public static final int PID_SLOT = 0;
    public static final double ARBITRARY_FEED_FORWARD_UP_IN_PERCENT = 0.4; // FIXME find arbitrary feed foward for hood
    public static final double ARBITRARY_FEED_FORWARD_DOWN_IN_PERCENT = -0.2; // FIXME find arbitrary feed foward for hood

    public static final double LOW_ANGLE = .2; // FIXME udpate once the value is known
    public static final double HIGH_ANGLE = 1.0147; // FIXME udpate once the value is known
    public static final double HOOD_DEGREES_TO_HOOD_ENCODER = 0; // encodervalue/degrees ratio
    public static final double POSITION_TOLERANCE = .1;

  }

  public static class LimelightConstants {
    public static final double HUB_H = 104;
    public static final double ROBOT_H = 21.25;
    public static final double GRAV_CONST_FT = -32.17519788;
    public static final double Flywheel_Radius_IN = 2;
    public static final double Velocity_Multiplier = 2;
    public static final double Ticks_Per_One_Rotation = 2048;
    public static final int LIMELIGHT_ANGLE_OFFSET = 0;
    public static final int LIMELIGHT_MOUNT_ANGLE = 50; 
    public static final int D2_D1_OFFSET_IN = 24;
    public static final int H2_H1_OFFSET_IN = -24;
    public static final int DISTANCE_TOLERANCE = 12;
    // 203" from center of hub to center of lauchpad;
    //  26" from edge of hub to center of hub;
    //  3" from center of launch pad to bumpers;
    //  18" from bumpers to center of robot;
    //  7.5" from center of robot to Limeligtht
    public static final int HUB_LAUNCHPAD_DISTANCE = 149; 
    public static final int HUB_WALL_DISTANCE = 111;

  }

  public static final class StorageConstants {
    public static final double OUTTAKE_POWER = -.8;
    public static final int SHOOTER_SENSOR = 1;
    public static final int COLLECTOR_SENSOR = 0;
    public static final int STORAGE_MOTOR_ID = 4;
    public static final double STORAGE_DEFAULT_SPEED = 0.6;
    public static final int STORAGE_CAMERA_PORT = 0;
    public static final int TIMEOUT_MS = 30;
  }

  public static class ElevatorConstants {

    public static final double MAX_ELEVATOR_HEIGHT = 283089;
    public static final double MIN_DETACH_ENCODER_HEIGHT = 270000; // FIXME: set back to 0 or determine appropriate value after climber rebuild
    public static final double MIN_ELEVATOR_ENCODER_HEIGHT = 0;
    public static final double MID_RUNG_HEIGHT = 277129;
    public static final double REACH_TO_NEXT_RUNG_HEIGHT = 0; //    // FIXME: not sure if this is needed; use MAX_ELEVATOR_HEIGHT?
    public static final double PITCH_SETPOINT = 0;    // FIXME
    public static final double PITCH_TOLERANCE = 0;   // FIXME
    public static final int ELEVATOR_POSITION_TOLERANCE = 500;
    public static final double ARBITRARY_FEED_FORWARD_EXTEND = .02;
    public static final double ARBITRARY_FEED_FORWARD_RETRACT = -0.07;
    public static final double DEFAULT_MOTOR_POWER = 0.5; //FIXME TUNE

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final Gains GAINS_POSITION = new Gains(0.4 /* kP */, 0 /* kI */, 0 /* kD */, 0 /* kF */,
        0 /* kIZone */, 1.00 /* max output */);
    public static final double MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
    public static final double ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
    public static final int SCURVE_STRENGTH = 0;

    // CAN ID
    public static final int PIGEON_ID = 18;
    public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = 22;
    public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = 19;
    public static final int CLIMBER_CAMERA_PORT = 0;
  }

  public class SecondMechanismConstants {
    public static final int PNEUMATIC_HUB_CAN_ID = 20;
    public static final int PNEUMATIC_CHANNEL = 1;
  }

  public static final class JoystickConstants { // FIXME: make consistent with software feature sheet and use in RobotContainer
    public static final int CLIMBER_UP = 2;
    public static final int CLIMB_2 = 1;
    public static final int CLIMB_3 = 7;
    public static final int CLIMB_4 = 8;
    public static final int FENDER = 9;
    public static final int FIELD_WALL = 6;
    public static final int LAUNCHPAD = 5;
    public static final int TARMAC = 4;
    public static final int SHOOT = 3;
    public static final int SHOOT_SLOW = 10;
    public static final int UNASSIGNED = 11;
    public static final int CLIMB_CAM = 12;

    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_LB = 5;
    public static final int BUTTON_RB = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int LEFT_JOYSTICK_BUTTON = 9;
    public static final int RIGHT_JOYSTICK_BUTTON = 10;

  }
}
