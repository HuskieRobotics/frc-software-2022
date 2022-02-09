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

import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Feeder extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX talonSRX4;
private DigitalInput cargoDetection;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    private NetworkTableEntry cargoDetectedNT;
    /**
    *
    */
    public Feeder() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
talonSRX4 = new WPI_TalonSRX(4);
 
 

cargoDetection = new DigitalInput(0);
 addChild("CargoDetection", cargoDetection);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        this.cargoDetectedNT = Shuffleboard.getTab("Intake")
                .add("FeederCargoDetected", false)
                .getEntry();

        Shuffleboard.getTab("Intake").add("AdvanceCargoCommand", new AdvanceCargoCommand(this));
        Shuffleboard.getTab("Intake").add("TransferCargoToShooterCommand", new TransferCargoToShooterCommand(this));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        this.cargoDetectedNT.setBoolean(this.isCargoAdvanced());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void advanceBelt() {
        this.talonSRX4.set(Constants.FEEDER_MOTOR_POWER);
    }

    public void stopBelt() {
        this.talonSRX4.set(0.0);
    }

    public boolean isCargoTransferred() {
        return !this.cargoDetection.get();  // return true if cargo is not detected
    }

    public boolean isCargoAdvanced() {
        return this.cargoDetection.get();   // return true if cargo is detected
    }

}

