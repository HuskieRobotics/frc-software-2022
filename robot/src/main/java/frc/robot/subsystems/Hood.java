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
import static frc.robot.Constants.HoodConstants.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import java.util.Map;

//import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Hood extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax hoodMotor;
    private SparkMaxPIDController m_pidController;
    private NetworkTableEntry motorPowerNT;
    private NetworkTableEntry FConstantHoodNT; 
    private NetworkTableEntry PConstantHoodNT; 
    private NetworkTableEntry IConstantHoodNT; 
    private NetworkTableEntry DConstantHoodNT; 
    private NetworkTableEntry rotationsNT;
    private NetworkTableEntry setRotationsNT;
    private RelativeEncoder hoodEncoder;
    private LimelightMath lm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
   
    /**
    *
    */
    public Hood() {
    
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    hoodMotor = new CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    hoodMotor.restoreFactoryDefaults();


    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = hoodMotor.getPIDController();

    // Encoder object created to display position values
    hoodEncoder = hoodMotor.getEncoder(); 
    hoodEncoder.setPosition(0.0);

    m_pidController.setP(KP);
    m_pidController.setI(KI);
    m_pidController.setD(KD);
    m_pidController.setIZone(KIz);
    m_pidController.setFF(KFF);
    m_pidController.setOutputRange(K_MIN_OUTPUT, K_MAX_OUTPUT);
    


        this.motorPowerNT = Shuffleboard.getTab("HoodTuning")
                .add("motor power", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();
                
        this.FConstantHoodNT = Shuffleboard.getTab("HoodTuning")
                .add("kF", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.PConstantHoodNT = Shuffleboard.getTab("HoodTuning")
                .add("kP", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.IConstantHoodNT = Shuffleboard.getTab("HoodTuning")
                .add("kI", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.DConstantHoodNT = Shuffleboard.getTab("HoodTuning")
                .add("kD", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();
        this.rotationsNT = Shuffleboard.getTab("HoodTuning")
                .add("rotations", 0.0)
                .getEntry();
        this.setRotationsNT = Shuffleboard.getTab("HoodTuning")
                .add("setRotations", 0.0)
                .getEntry();



        Shuffleboard.getTab("Hood").addNumber("HoodEncoderReading", this::getHoodEncoderPosition);
        Shuffleboard.getTab("Hood").addNumber("HoodLimelightRotations", this::getHoodSetpointLimeLight);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {

        if (TUNING) {

            // when tuning, we first set motor power and check the resulting velocity
            // once we have determined our feedforward constant, comment the following lines
            double motorPower = this.motorPowerNT.getDouble(0.0);
            this.setHoodMotorPower(motorPower);
            double hoodSetPoint = this.setRotationsNT.getDouble(0.0);
            this.setHoodSetpoint(hoodSetPoint);
            
            // uncomment these lines after determining feed forward
            // m_pidController.setFF(FConstantHoodNT.getDouble(0.0));
            // m_pidController.setP(PConstantHoodNT.getDouble(0.0));
            // m_pidController.setI(IConstantHoodNT.getDouble(0.0));
            // m_pidController.setD(DConstantHoodNT.getDouble(0.0));
            
            // double rotations = this.rotationsNT.getDouble(0.0);
            // this.setHoodSetpoint(rotations);
        }
       


        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public double getHoodEncoderPosition() {
        return hoodEncoder.getPosition();
    }

    
    public double getHoodSetpointLimeLight() {
        
        double a = lm.getIdealHoodA();
        return a * HOOD_DEGREES_TO_HOOD_ENCODER; //will this return rotations?
    }

    private void setHoodMotorPower(double pwr) {
        this.hoodMotor.set(pwr);
    }

    public void setHoodSetpoint(double rotations){

        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }


}

