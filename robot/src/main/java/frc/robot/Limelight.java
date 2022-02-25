package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;




public class Limelight {
    
    private NetworkTableEntry limelightDistanceInNT;
    private NetworkTableEntry velocityNT;
    private NetworkTableEntry shooterAngleNT;


    double vx;
    double vy;
    double d2;
    double d1;
    double h2;
    double h1;

    public Limelight() {
    
        d2 = getLimelightDistanceIn();
        d1 = d2 + LimelightConstants.D2_D1_OFFSET;
        h2 = LimelightConstants.HUB_H + 12;
        h1 = h2 + LimelightConstants.H2_H1_OFFSET;
    }

    private void resetVXVY() {
        double g = LimelightConstants.GRAV_CONST;
        vx = Math.sqrt(-(d1*d1)*d2*g+d1*(d2*d2)*g)/(Math.sqrt(2)*Math.sqrt(-d2*h1+d1*h2));
        vy = (g*((d2*d2*h1)-(d1*d1*h2)))/(Math.sqrt(d1*d2*(-d1+d2)*g)*Math.sqrt(-2*d2*h1+2*d1*h2));
    }
    public double getLimelightDistanceIn() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        double d = (LimelightConstants.HUB_H-LimelightConstants.ROBOT_H)/Math.tan(44-LimelightConstants.LIMELIGHT_ANGLE_OFFSET+ty);
        return d;
    }
    public double getIdealVelocity() {
        // ** **** ** * *********** ******* ******* *** **** **** *** *** ******* *** **** *** **** ** * ***
        
        double v;
    
        resetVXVY();
        v = Math.sqrt((vx*vx)+(vy*vy));

        return v;
    }

    public double getIdealHoodA() {
        double a;

        resetVXVY();
        a = Math.atan(vy/vx);

        return a;
        

    }
}
