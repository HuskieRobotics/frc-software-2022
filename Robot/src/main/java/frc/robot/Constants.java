package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  /* DRIVETRAIN CONSTANTS */

  public static final class DrivetrainConstants {

    private DrivetrainConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(118.0371);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(102.9968);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(172.7051);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(40.3335);

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.5969; // 23.5 inches

    public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
    public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters
    public static final double EVASIVE_ROTATION_COG_SHIFT_MAGNITUDE =
        0.707; // a bit beyond the bumper permimeter (meters)
    public static final double COG_OFFSET = 45;

    public static final int PIGEON_ID = 18;
    public static final int TIMEOUT_MS = 30;

    /* Limelight */
    public static final String LIMELIGHT_NETWORK_TABLE_NAME = "limelight";
    public static final double LIMELIGHT_F = 0.1;
    public static final double LIMELIGHT_P = 0.2;
    public static final double LIMELIGHT_I = 0.50;
    public static final double LIMELIGHT_ALIGNMENT_TOLERANCE = 1.0;
    public static final double LIMELIGHT_LAUNCHPAD_ALIGNMENT_TOLERANCE = .6;
    public static final double LIMELIGHT_AIM_TOLERANCE = 3; // inches
    public static final int AIM_SETPOINT_COUNT = 2;
    public static final double LIMELIGHT_SLOPE = 23.08;
    public static final double LIMELIGHT_Y_COMPONENT = 4596.34;

    /* Rev Hubs */
    public static final int POWER_DISTRIBUTION_HUB_ID = 21;
    public static final int PNEUMATICS_HUB_ID = 20;
  }

  public static final class CollectorConstants {

    private CollectorConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double OUTTAKE_POWER = -0.7;
    public static final int COLLECTOR_MOTOR_ID = 5;
    public static final int PEUNAMATICS_HUB_CAN_ID = 20;
    public static final int COLLECTOR_SOLENOID_CHANNEL = 0;
    public static final double COLLECTOR_DEFUALT_SPEED = 0.9;
    public static final int TIMEOUT_MS = 30;
  }

  public static final class AutoConstants {

    private AutoConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    // from sysId tool
    public static final double S_VOLTS = 0.55493;
    public static final double V_VOLT_SECONDS_PER_METER = 2.3014;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.12872;

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

    public static final double PX_CONTROLLER = 2.2956; // from sysId tool
    public static final double PY_CONTROLLER = 2.2956; // from sysId tool
    public static final double P_THETA_CONTROLLER = 4.9;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static class FlywheelConstants {

    private FlywheelConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int VELOCITY_TOLERANCE = 300;
    public static final int LEFT_FLYWHEELMOTOR_CANID = 1;
    public static final int RIGHT_FLYWHEELMOTOR_CANID = 2;
    public static final double VELOCITY_PID_P = 0.18;
    public static final double VELOCITY_PID_I = 0.0;
    public static final double VELOCITY_PID_D = 0.0;
    public static final double VELOCITY_PID_F = 0.0513;
    public static final double VELOCITY_PID_I_ZONE = 0.0;
    public static final double VELOCITY_PID_PEAK_OUTPUT = 1.0;
    public static final int MAX_FLYWHEEL_VELOCITY = 18650; // ticks per 100 ms
    public static final int NEAR_WALL_SHOT_VELOCITY = 7000; // ticks per 100 ms
    public static final int WALL_SHOT_VELOCITY = 7182; // ticks per 100 ms
    public static final int FENDER_SHOT_VELOCITY = 7799; // ticks per 100 ms
    public static final int LAUNCH_PAD_VELOCITY = 8182; // ticks per 100 ms
    public static final int SHOOT_SLOW_VELOCITY = 4000; // ticks per 100 ms
    public static final int SHOOT_STEAL_VELOCITY = 5500; // ticks per 100 ms
    public static final double REVERSE_POWER = -0.2;
    public static final int SETPOINTCOUNT = 5;
  }

  public static class LimelightConstants {

    private LimelightConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double HUB_H = 104; // inches
    public static final double ROBOT_H = 21.25; // inches
    public static final double GRAV_CONST_FT = -32.17519788;
    public static final double FLYWHEEL_RADIUS_INCHES = 2; // inches
    public static final double VELOCITY_MULTIPLIER = 2;
    public static final double TICKS_PER_ROTATION = 2048;
    public static final int LIMELIGHT_ANGLE_OFFSET = -2; // degrees
    public static final int LIMELIGHT_MOUNT_ANGLE = 45; // degrees
    public static final int D2_D1_OFFSET_IN = 24; // inches
    public static final int H2_H1_OFFSET_IN = -24; // inches
    public static final int DISTANCE_TOLERANCE = 12; // inches
    // 203" from center of hub to center of lauchpad
    //  26" from edge of hub to center of hub
    //  3" from center of launch pad to bumpers
    //  18" from bumpers to center of robot
    //  7.5" from center of robot to Limeligtht
    public static final int HUB_LAUNCHPAD_DISTANCE = 149; // inches
    public static final int EDGE_TO_CENTER_HUB_DISTANCE = 26 + 8; // inches
    public static final int HUB_WALL_DISTANCE = 130; // inches
    public static final int AUTO_SHOT_HUB_FAR_DISTANCE = 124; // inches
    public static final int AUTO_SHOT_HUB_CLOSE_DISTANCE = 68; // inches
  }

  public static final class StorageConstants {

    private StorageConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double OUTTAKE_POWER = -.8;
    public static final int SHOOTER_SENSOR = 1;
    public static final int COLLECTOR_SENSOR = 0;
    public static final int STORAGE_MOTOR_ID = 4;
    public static final double STORAGE_DEFAULT_SPEED = 0.7;
    public static final int STORAGE_CAMERA_PORT = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int WAIT_FOR_SHOT_DELAY = 10;
    public static final int INDEXING_FORWARD_DELAY = 16;
    public static final int INDEXING_BACKWARD_DURATION = 3;
  }

  public static class ElevatorConstants {

    private ElevatorConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double MIN_ELEVATOR_ENCODER_HEIGHT = 0;
    public static final double TRANSFER_TO_SECONDARY_HEIGHT = 25150;
    public static final double LOW_RUNG_HEIGHT = 150786;
    public static final double REACH_JUST_BEFORE_NEXT_RUNG = 185227;
    public static final double MID_RUNG_HEIGHT = 253338;

    public static final double NEXT_RUNG_HEIGHT = 227499;
    public static final double LATCH_HIGH_RUNG_ENCODER_HEIGHT = 238151;
    public static final double LATCH_TRAVERSE_RUNG_ENCODER_HEIGHT = 171592;
    public static final double REACH_TO_NEXT_RUNG_HEIGHT = 265674;
    public static final double MAX_ELEVATOR_HEIGHT = 272631;
    public static final double TICKS_PER_INCH = 8874.266;

    public static final double RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG = 0.040;

    public static final int ELEVATOR_POSITION_TOLERANCE = 1000;
    public static final double ARBITRARY_FEED_FORWARD_EXTEND = .02;
    public static final double ARBITRARY_FEED_FORWARD_RETRACT = -0.07;
    public static final double DEFAULT_MOTOR_POWER = 0.5;

    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final double POSITION_PID_P = 0.4;
    public static final double POSITION_PID_I = 0.0;
    public static final double POSITION_PID_D = 0.0;
    public static final double POSITION_PID_F = 0.0;
    public static final double POSITION_PID_I_ZONE = 0.0;
    public static final double POSITION_PID_PEAK_OUTPUT = 1.0;
    public static final double SLOW_PEAK_OUTPUT = 0.15;
    public static final double MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
    public static final double ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
    public static final int SCURVE_STRENGTH = 0;

    public static final int SAMPLE_WINDOW_WIDTH = 6;
    public static final double EPSILON = 0.001;

    // CAN ID
    public static final int PIGEON_ID = 18;
    public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = 22;
    public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = 19;
    public static final int CLIMBER_CAMERA_PORT = 0;
  }

  public class SecondaryArmConstants {

    private SecondaryArmConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int PNEUMATIC_HUB_CAN_ID = 20;
    public static final int PNEUMATIC_CHANNEL = 1;
  }

  public static final class JoystickConstants {

    private JoystickConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int CLIMBER_UP = 2;
    public static final int CLIMB_2 = 1;
    public static final int CLIMB_3 = 7;
    public static final int CLIMB_4 = 8;
    public static final int LIMELIGHT_AIM_TOGGLE = 9;
    public static final int FIELD_WALL = 6;
    public static final int LAUNCHPAD = 5;
    public static final int SECONDARY = 4;
    public static final int AUTO_AIM_AND_SHOOT = 3;
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
