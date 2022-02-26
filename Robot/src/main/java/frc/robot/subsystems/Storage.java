// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import static frc.robot.Constants.*;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Storage extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX storage4;
private DigitalOutput collectorSensor0;
private DigitalOutput shooterSensor1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private boolean isRetracted;
    private NetworkTableEntry shootSensorReadingNT;
    private NetworkTableEntry collectorSensorReadingNT;
    private NetworkTableEntry storagePowerSetPointNT;
    /**
    *
    */
    public Storage() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    storage4 = new WPI_TalonSRX(StorageConstants.STORAGE_MOTOR_ID);
 
 

    collectorSensor0 = new DigitalOutput(StorageConstants.COLLECTOR_SENSOR);
    addChild("Collector Sensor 0",collectorSensor0);
 

    shooterSensor1 = new DigitalOutput(StorageConstants.SHOOTER_SENSOR);
    addChild("Shooter Sensor 1",shooterSensor1);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    this.isRetracted = true;
    this.collectorSensorReadingNT = Shuffleboard.getTab("Storage")
        .add("Collector Sensor Unblocked", true)
        .getEntry();
    this.shootSensorReadingNT = Shuffleboard.getTab("Storage")
        .add("Shooter Sensor Unblocked", true)
        .getEntry();
    this.storagePowerSetPointNT = Shuffleboard.getTab("Storage")
        .add("Storage Power", 0.0)
        .getEntry();
    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.collectorSensorReadingNT.setBoolean(this.collectorSensorUnblocked());
        this.shootSensorReadingNT.setBoolean(this.shooterSensorUnblocked());
        if(TUNING){
        double storagePower = this.storagePowerSetPointNT.getDouble(0.0); 
        this.setStoragePower(storagePower);
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setStoragePower(double power){
        this.storage4.set(ControlMode.PercentOutput, power);
    }

    public boolean shooterSensorUnblocked(){
        return this.shooterSensor1.get();
    }

    public boolean collectorSensorUnblocked(){
        return this.collectorSensor0.get();
    }

    public double getStoragePower(){
        return this.storage4.get();
    }
    
    


}

