package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionBoxConstants;

public class VisionBox extends SubsystemBase {

    private NetworkTableEntry txListNT;
    private NetworkTableEntry tyListNT;
    private NetworkTable configNT;
    private DrivetrainSubsystem drivetrainSubsystem;
    private final Field2d ballField = new Field2d();
    
    public VisionBox(DrivetrainSubsystem m_drivetrain) {
        /*
        tx and ty and camera-relative measures of angle (where x is to the left and right of the camera and y is up and down).
        In the robot relative coordinate system, tx is a rotation about the z axis from +x to +y. ty is a rotation about the y axis from +x to +z.
        This leads to some counter-intuitive logic where tx is used to find a y coordinate and ty is used to find the x.
        */
        this.drivetrainSubsystem = m_drivetrain; //need the drivetrain to get pose
        this.txListNT = NetworkTableInstance.getDefault().getTable("VisionBox").getSubTable("output").getEntry("tx_list");
        this.tyListNT = NetworkTableInstance.getDefault().getTable("VisionBox").getSubTable("output").getEntry("ty_list");
        this.configNT = NetworkTableInstance.getDefault().getTable("VisionBox").getSubTable("config");
        
        //put field data on SmartDashboard (COMMENT THIS OUT WHEN DONE TESTING)
        SmartDashboard.putData("Field", ballField);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // updateBallColorConstants();

        // show ball and robot pose on map (COMMENT THIS OUT WHEN DONE WITH TESTING)
        // if (drivetrainSubsystem.getPose() != null && getFirstBallTransform2d() != null) {
        //     ballField.setRobotPose(drivetrainSubsystem.getPose());
        //     ballField.getObject("ball").setPose(drivetrainSubsystem.getPose().plus(getFirstBallTransform2d()));
        // }

    }
    /**
     * returns a list, with each entry being a ball's angle relative to the camera in the horizontal direction in radians
     * @param defaultList a default list to be returned
     * @return the list
     */
    public double[] getTxList(double[] defaultList) {
        return this.txListNT.getDoubleArray(defaultList);
    }

    public double[] getTxList() {
        return getTxList(new double[0]);
    }

    /**
     * returns a list, with each entry being a ball's angle relative to the camera in the vertical direction in radians
     * @param defaultList a default list to be returned
     * @return the list
     */
    public double[] getTyList(double[] defaultList) {
        return this.tyListNT.getDoubleArray(defaultList);
    }
    
    public double[] getTyList() {
        return getTyList(new double[0]);
    }

    /**
     * get the first ball's angle relative to the camera in the horizontal direction (positive theta is to the left of the image) in radians
     * @return the angle relative to the robot or null if no balls are detected
     */
    public Double getFirstBallTx() {
        double[] txList = getTxList();
        if (txList.length == 0) {
            return null;
        } else {
            return txList[0];
        }
    }

    /**
     * get the first ball's angle relative to the camera in the vertical direction (positive theta is to the top of the image) in radians
     * @return the angle relative to the robot or null if no balls are detected
     */
    public Double getFirstBallTy() {
        double[] tyList = getTyList();
        if (tyList.length == 0) {
            return null;
        } else {
            return tyList[0];
        }
    }

    /**
     * Get the transform from the robot to the first ball. This includes rotational and translational position data.
     * @return the translation or null if no balls are detected
     */
    public Transform2d getFirstBallTransform2d() {

        Double tx = getFirstBallTx();
        Double ty = getFirstBallTy();
        
        //check if the first ball exists, if not return null
        if (tx == null) return null;

        //determine the ball's distance from the robot in the x direction
        double x = VisionBoxConstants.CAMERA_HEIGHT_METERS / Math.abs(Math.tan(Math.toRadians(VisionBoxConstants.CAMERA_ANGLE_DEGREES)-ty));

        //using the direction in the radial direction and tx, calculate the distance in the y direction
        double y = x * Math.tan(tx);
        
        Rotation2d rotation = new Rotation2d(0); //we dont care which way the ball is rotated
        Translation2d translation = new Translation2d(x, y);

        return new Transform2d(translation, rotation);

    }

    /**
     * Set constants for visionBox based on alliance color in FMS
     */
    public void updateBallColorConstants() {
        if (true) { //check alliance color //NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false)
            //RED BALL CONSTANTS
            configNT.getEntry("lowerHue").setDouble(159);
            configNT.getEntry("lowerSaturation").setDouble(127);
            configNT.getEntry("lowerValue").setDouble(25);
            configNT.getEntry("upperHue").setDouble(9);
            configNT.getEntry("upperSaturation").setDouble(255);
            configNT.getEntry("upperValue").setDouble(255);
        } else {
            //BLUE BALL CONSTANTS
            configNT.getEntry("lowerHue").setDouble(84);
            configNT.getEntry("lowerSaturation").setDouble(57);
            configNT.getEntry("lowerValue").setDouble(115);
            configNT.getEntry("upperHue").setDouble(128);
            configNT.getEntry("upperSaturation").setDouble(255);
            configNT.getEntry("upperValue").setDouble(255);
        }
    }

}
