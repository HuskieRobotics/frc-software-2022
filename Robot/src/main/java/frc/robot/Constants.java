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

  public static final boolean TUNING = false;
  public static final boolean COMMAND_LOGGING = false;
  public static final double BROWNOUT_VOLTAGE_LIMIT =
      7.5; // volts; FIXME: tune this as this may be too high, 6.5?
  public static final int LIMIT_CURRENT_DRAW_PERIOD = 50; // FIXME: tune

  /* DRIVETRAIN CONSTANTS */

  public static final class DrivetrainConstants {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // 16;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // 15;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8; // 17;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(118.0371);
    // 149.589843
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13; // 10;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14; // 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
        -Math.toRadians(102.9968); // 103.2715);
    // 172.96875
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // 13;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; // 12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(172.7051);
    // 103.27148
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16; // 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15; // 6;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17; // 8;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
        -Math.toRadians(40.3335); // 149.5020);
    // 117.333984

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches;
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

    public static final double CURRENT_LIMIT_FACTOR = 0.5; // FIXME: this is extreme; tune it

    /* Limelight */
    public static final double LIMELIGHT_F = 0.1;
    public static final double LIMELIGHT_P = 0.2;
    public static final double LIMELIGHT_I = 0.50;
    public static final double LIMELIGHT_ALIGNMENT_TOLERANCE = 1.0;
    public static final double LIMELIGHT_LAUNCHPAD_ALIGNMENT_TOLERANCE = .6;
    public static final double LIMELIGHT_AIM_TOLERANCE = 3; // inches
    public static final int AIM_SETPOINT_COUNT = 2;
    public static final double LIMELIGHT_SLOPE = 23.08;
    public static final double LIMELIGHT_Y_COMPONENT = 4596.34;
  }

  public static final class CollectorConstants {
    public static final double OUTTAKE_POWER = -0.7; // FIX_ME
    public static final int COLLECTOR_MOTOR_ID = 5;
    public static final int PEUNAMATICS_HUB_CAN_ID = 20;
    public static final int COLLECTOR_SOLENOID_CHANNEL = 0;
    public static final double COLLECTOR_DEFUALT_SPEED =
        0.9; // FIX_ME change this to desired speed; was 0.7
    public static final int TIMEOUT_MS = 30;
  }

  public static final class AutoConstants {

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
    public static final double P_THETA_CONTROLLER =
        4.9; // tune after verifying non-holonomic motion

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static class FlywheelConstants {
    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int VELOCITY_TOLERANCE = 300;
    public static final int LEFT_FLYWHEELMOTOR_CANID = 1;
    public static final int RIGHT_FLYWHEELMOTOR_CANID = 2;
    public static final Gains GAINS_VELOCITY =
        new Gains(
            0.18 /* kP */,
            0 /* kI */,
            0 /* kD */,
            .0513 /* kF */,
            0 /* kIzone */,
            1.00 /* kPeakOutput */);
    // kp:0.5
    // kf: .0438
    public static final int MAX_FLYWHEEL_VELOCITY = 18650;
    public static final int NEAR_WALL_SHOT_VELOCITY = 7000;
    public static final int WALL_SHOT_VELOCITY = 7182;
    public static final int FENDER_SHOT_VELOCITY = 7799;
    public static final int LAUNCH_PAD_VELOCITY = 8182;
    public static final int SHOOT_SLOW_VELOCITY = 4000;
    public static final int SHOOT_STEAL_VELOCITY = 5500;
    public static final double REVERSE_POWER = -0.2;
    public static final int SETPOINTCOUNT = 5;
  }

  public static class HoodConstants {
    public static final int HOOD_MOTOR_ID = 3;
    public static final double KP = 0; // FIXME find hood pid p value
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KI_Z = 0;
    public static final double KFF = 0;
    public static final double K_MAX_OUTPUT = 1;
    public static final double K_MIN_OUTPUT = -1;
    public static final int TIMEOUT_MS = 30;

    public static final int PID_SLOT = 0;
    public static final double ARBITRARY_FEED_FORWARD_UP_IN_PERCENT =
        0.4; // FIXME find arbitrary feed foward for hood
    public static final double ARBITRARY_FEED_FORWARD_DOWN_IN_PERCENT =
        -0.2; // FIXME find arbitrary feed foward for hood

    public static final double LOW_ANGLE = .2; // FIXME udpate once the value is known
    public static final double HIGH_ANGLE = 1.0147; // FIXME udpate once the value is known
    public static final double HOOD_DEGREES_TO_HOOD_ENCODER = 0; // encodervalue/degrees ratio
    public static final double POSITION_TOLERANCE = .1;
  }

  public static class LimelightConstants {
    public static final double HUB_H = 104;
    public static final double ROBOT_H = 21.25;
    public static final double GRAV_CONST_FT = -32.17519788;
    public static final double FLYWHEEL_RADIUS_INCHES = 2;
    public static final double VELOCITY_MULTIPLIER = 2;
    public static final double TICKS_PER_ROTATION = 2048;
    public static final int LIMELIGHT_ANGLE_OFFSET = -2;
    public static final int LIMELIGHT_MOUNT_ANGLE = 45;
    public static final int D2_D1_OFFSET_IN = 24;
    public static final int H2_H1_OFFSET_IN = -24;
    public static final int DISTANCE_TOLERANCE = 12;
    // 203" from center of hub to center of lauchpad
    //  26" from edge of hub to center of hub
    //  3" from center of launch pad to bumpers
    //  18" from bumpers to center of robot
    //  7.5" from center of robot to Limeligtht
    public static final int HUB_LAUNCHPAD_DISTANCE = 149;
    public static final int EDGE_TO_CENTER_HUB_DISTANCE = 26 + 8;
    public static final int HUB_WALL_DISTANCE = 130;
    public static final int AUTO_SHOT_HUB_FAR_DISTANCE = 124;
    public static final int AUTO_SHOT_HUB_CLOSE_DISTANCE = 68;
  }

  public static final class StorageConstants {
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

    public static final double MIN_ELEVATOR_ENCODER_HEIGHT = 0;
    public static final double TRANSFER_TO_SECONDARY_HEIGHT = 25150;
    public static final double LOW_RUNG_HEIGHT = 150786;
    public static final double REACH_JUST_BEFORE_NEXT_RUNG = 185227;
    public static final double MID_RUNG_HEIGHT = 253338; // 259788;//269560;

    public static final double NEXT_RUNG_HEIGHT = 227499;
    public static final double LATCH_HIGH_RUNG_ENCODER_HEIGHT = 238151;
    public static final double LATCH_TRAVERSE_RUNG_ENCODER_HEIGHT =
        171592; // 136095;//subtract a foot//242587;//240369;
    public static final double REACH_TO_NEXT_RUNG_HEIGHT = 265674;
    public static final double MAX_ELEVATOR_HEIGHT = 272631; // 274560;
    public static final double TICKS_PER_INCH = 8874.266;

    public static final double RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG = 0.040;

    public static final int ELEVATOR_POSITION_TOLERANCE = 1000;
    public static final double ARBITRARY_FEED_FORWARD_EXTEND = .02;
    public static final double ARBITRARY_FEED_FORWARD_RETRACT = -0.07;
    public static final double DEFAULT_MOTOR_POWER = 0.5;

    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final Gains GAINS_POSITION =
        new Gains(
            0.4 /* kP */,
            0 /* kI */,
            0 /* kD */,
            0 /* kF */,
            0 /* kIZone */,
            1.00 /* max output */);
    public static final double SLOW_PEAK_OUTPUT = 0.15;
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

  public static final class JoystickConstants {
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
