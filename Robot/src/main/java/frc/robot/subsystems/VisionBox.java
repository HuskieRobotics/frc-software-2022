package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionBoxConstants;

public class VisionBox extends SubsystemBase {

    private NetworkTableEntry txListNT;
    private NetworkTableEntry tyListNT;

    public VisionBox() {
        this.txListNT = NetworkTableInstance.getDefault().getTable("visionBox").getSubTable("output").getEntry("tx_list");
        this.tyListNT = NetworkTableInstance.getDefault().getTable("visionBox").getSubTable("output").getEntry("ty_list");
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
     * get the first ball's angle relative to the camera in the horizontal direction in radians
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
     * get the first ball's angle relative to the camera in the vertical direction in radians
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

        //determine the ball's distance from the robot in the radial direction (y when in robot relative coordinates)
        double y = VisionBoxConstants.CAMERA_HEIGHT_METERS / Math.tan(ty);

        //using the direction in the radial direction and tx, calculate the distance in the lateral direction (x when in robot relative coordinates)
        double x = y / Math.tan(tx);
        
        Rotation2d rotation = new Rotation2d(0); //we dont care which way the ball is rotated
        Translation2d translation = new Translation2d(x, y);

        return new Transform2d(translation, rotation);

    }

    /**
     * Set which vision pipeline for the coprocessor to use
     * @param pipelineId the id of the selected pipeline
     */
    public void setPipeline(String pipelineId) {
        //TODO: implement this
    }

}
