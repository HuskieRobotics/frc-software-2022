package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;




public class LimelightMath extends SubsystemBase{
    
    private NetworkTableEntry shooterAngleNT;
    private NetworkTableEntry limelightDistanceInNT;
    private NetworkTableEntry velocityNT;
    private NetworkTableEntry tyNT;
    private NetworkTableEntry getvx;
    private NetworkTableEntry getvy;
    private double ty;
    double vx;
    double vy;
    double h2;
    double h1;

    public LimelightMath() {
    
        
        h2 = LimelightConstants.HUB_H + 12;
        h1 = h2 + LimelightConstants.H2_H1_OFFSET_IN;
        this.shooterAngleNT= Shuffleboard.getTab("LimeTuning")
            .add("shooterAng", 0.0)
            .getEntry();
        this.limelightDistanceInNT= Shuffleboard.getTab("LimeTuning")
            .add("DistanceOutput", 0.0)
            .getEntry();
        this.velocityNT= Shuffleboard.getTab("LimeTuning")
                .add("IdealVelocity", 0.0)
                .getEntry();
        this.tyNT = Shuffleboard.getTab("LimeTuning")
                .add("ty", 0.0)
                .getEntry();

        this.getvx = Shuffleboard.getTab("LimeTuning")
                .add("vx", 0.0)
                .getEntry();
        
        this.getvy = Shuffleboard.getTab("LimeTuning")
                .add("vy", 0.0)
                .getEntry();


    }

    @Override
    public void periodic() {
        this.velocityNT.setDouble(this.getIdealVelocity());
        this.shooterAngleNT.setDouble(this.getIdealHoodA());
        this.limelightDistanceInNT.setDouble(this.getLimelightDistanceIn());
        this.tyNT.setDouble(this.ty);
        this.getvx.setDouble(this.vx);
        this.getvy.setDouble(this.vy);
        
    }
    
    private void resetVXVY() {
        double g = LimelightConstants.GRAV_CONST_FT*12;
        double d2= this.getLimelightDistanceIn();
        double d1= d2 + LimelightConstants.D2_D1_OFFSET_IN;
        vx = Math.sqrt(-(d1*d1)*d2*g+d1*(d2*d2)*g)/(Math.sqrt(2)*Math.sqrt(-d2*h1+d1*h2));
        vy = (g*((d2*d2*h1)-(d1*d1*h2)))/(Math.sqrt(d1*d2*(-d1+d2)*g)*Math.sqrt(-2*d2*h1+2*d1*h2));
    }
    public double getLimelightDistanceIn() {
        this.ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        
        
        
        double d = (LimelightConstants.HUB_H-LimelightConstants.ROBOT_H)/(Math.tan(Math.toRadians(43 *+LimelightConstants.LIMELIGHT_ANGLE_OFFSET+ ty)));

        
        return d;
    }
    public double getIdealVelocity() {  
        // ** **** ** * *********** ******* ******* *** **** **** *** *** ******* *** **** *** **** ** * ***
        
        double v;
    
        resetVXVY();
        v = Math.sqrt((vx*vx)+(vy*vy))*(LimelightConstants.Velocity_Multiplier)/(2*Math.PI*(LimelightConstants.Flywheel_Radius_IN))*(LimelightConstants.Ticks_Per_One_Rotation)/10;

        return v;
    }

    public double getIdealHoodA() {
        double a;

        resetVXVY();
        a = Math.toDegrees(Math.atan(vy/vx));

        return a;
        

    }
    
    public void ledOff(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void ledOn(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void ledSetDefaultState(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }
}