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


import frc.robot.Constants.CollectorConstants;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
    public class Collector extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX collector5;
    private Solenoid collectorPiston;
    private boolean isEnabled;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private NetworkTableEntry collectorMotorPowerNT;
    
    /**
    *
    */
    public Collector()  {
        isEnabled = false;
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    collector5 = new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_ID);
    collector5.setInverted(true);
 
 

    collectorPiston = new Solenoid(CollectorConstants.PEUNAMATICS_HUB_CAN_ID, PneumaticsModuleType.REVPH, CollectorConstants.COLLECTOR_SOLENOID_CHANNEL);
    addChild("Collector Piston", collectorPiston);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    this.collectorMotorPowerNT = Shuffleboard.getTab("Collector")
        .add("Collector speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
    Shuffleboard.getTab("Collector").add("deployCollector",  new InstantCommand(this :: deployCollectorPiston, this));
    Shuffleboard.getTab("Collector").add("retractCollector",  new InstantCommand(this :: retractCollectorPiston, this));
    Shuffleboard.getTab("Collector").addBoolean("enabled", this::isEnabled);
    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(TUNING){
        double collectorPower = this.collectorMotorPowerNT.getDouble(0.0);
        this.setCollectorPower(collectorPower);
        }

    }

    public void setCollectorPower(double power) { 
        this.collector5.set(ControlMode.PercentOutput, power);
    }

    public void disableCollector() {
        isEnabled = false;
        this.collector5.set(ControlMode.PercentOutput, 0);
        this.collectorPiston.set(false);
    }

    public void enableCollector(){
        isEnabled = true;
        this.collector5.set(ControlMode.PercentOutput, CollectorConstants.COLLECTOR_DEFUALT_SPEED);
        this.collectorPiston.set(true);
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public void deployCollectorPiston(){
       this.collectorPiston.set(true);
    }

    public void retractCollectorPiston(){
        this.collectorPiston.set(false);
    }

    public boolean isPistonExtended(){
        return this.collectorPiston.get();
    }

    

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

   

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

