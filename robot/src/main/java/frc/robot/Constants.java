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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final boolean TUNING = true;
    
    public static class FlywheelConstants{
        public static final int SLOT_INDEX = 0;
        public static final int PID_LOOP_INDEX = 0;
        public static final int TIMEOUT_MS = 30;
        public static final int VELOCITY_TOLERANCE = 500; //FIX_ME tune this should be how off our velocity can be 
        public static final int LEFT_FLYWHEELMOTOR_CANID = 2;
        public static final int RIGHT_FLYWHEELMOTOR_CANID = 1;
        


    public final static Gains GAINS_VELOCITY  = new Gains( .74, /* kP */  0, /* kI */   0,  /* kD */  .05265,   /* kF */     0,  /* kIzone */  1.00 /* kPeakOutput */);

   }
   public class HoodConstants{
       public static final int HOOD_MOTOR_ID = 3;
       public static final double KP = 0; //FIX_ME find hood pid p value 
       public static final double KI =0;
       public static final double KD =0;
       public static final double KIz =0;
       public static final double KFF =0;
        public static final double K_MAX_OUTPUT=0.2;
        public static final double K_MIN_OUTPUT=-0.2;

        public static final double HOOD_DEGREES_TO_HOOD_ENCODER=0; // encodervalue/degrees ratio

   }

   public class LimelightConstants {
       public static final double HUB_H = 104;
       public static final double ROBOT_H = 21.25;
       public static final double GRAV_CONST_FT = -32.17519788;
       public static final double Flywheel_Radius_IN = 2;
       public static final double Velocity_Multiplier = 2;
       public static final double Ticks_Per_One_Rotation = 2048;
       public static final int LIMELIGHT_ANGLE_OFFSET=-2;
       public static final int D2_D1_OFFSET_IN = 24;
       public static final int H2_H1_OFFSET_IN = -24;
       

   }

}

