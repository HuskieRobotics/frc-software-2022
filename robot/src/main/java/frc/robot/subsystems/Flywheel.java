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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LimelightMath;

import java.awt.geom.Point2D;

import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;

import java.lang.reflect.Field;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Flywheel extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonFX leftFlywheelMotor;
private WPI_TalonFX rightFlywheelMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private double velocitySetPoint;

    private NetworkTableEntry isAtSetpointNT;
    private NetworkTableEntry rightEncoderReadingNT;
    private NetworkTableEntry leftEncoderReadingNT;
    private NetworkTableEntry rightClosedLoopErrorNT;
    private NetworkTableEntry leftClosedLoopErrorNT;

    private NetworkTableEntry velocitySetPointNT;
    private NetworkTableEntry motorPowerNT;
    private NetworkTableEntry FConstantNT;
    private NetworkTableEntry PConstantNT;
    private NetworkTableEntry IConstantNT;
    private NetworkTableEntry DConstantNT;

    
    
    /**
    *
    */
    public Flywheel() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    leftFlywheelMotor = new WPI_TalonFX(2); 

    rightFlywheelMotor = new WPI_TalonFX(1);
 
    


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        this.leftFlywheelMotor.setInverted(true);
        
    
    /* Factory Default all hardware to prevent unexpected behaviour */
		rightFlywheelMotor.configFactoryDefault();
        leftFlywheelMotor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		rightFlywheelMotor.configNeutralDeadband(0.001);
        leftFlywheelMotor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        rightFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            FlywheelConstants.PID_LOOP_INDEX, 
											FlywheelConstants.TIMEOUT_MS);
        leftFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            FlywheelConstants.PID_LOOP_INDEX, 
											FlywheelConstants.TIMEOUT_MS);

		/* Config the peak and nominal outputs */
		rightFlywheelMotor.configNominalOutputForward(0, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.configNominalOutputReverse(0, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.configPeakOutputForward(1, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.configPeakOutputReverse(-1, FlywheelConstants.TIMEOUT_MS);
        leftFlywheelMotor.configNominalOutputForward(0, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.configNominalOutputReverse(0, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.configPeakOutputForward(1, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.configPeakOutputReverse(-1, FlywheelConstants.TIMEOUT_MS);

		/* Config the Velocity closed loop gains in slot0 */
		rightFlywheelMotor.config_kF(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kF, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.config_kP(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kP, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.config_kI(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kI, FlywheelConstants.TIMEOUT_MS);
		rightFlywheelMotor.config_kD(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kD, FlywheelConstants.TIMEOUT_MS);
        leftFlywheelMotor.config_kF(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kF, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.config_kP(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kP, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.config_kI(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kI, FlywheelConstants.TIMEOUT_MS);
		leftFlywheelMotor.config_kD(FlywheelConstants.PID_LOOP_INDEX, FlywheelConstants.GAINS_VELOCITY.kD, FlywheelConstants.TIMEOUT_MS);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // flywheelRight.setSensorPhase(true);
        this.velocitySetPoint = 0.0;

        this.isAtSetpointNT = Shuffleboard.getTab("Shooter")
                .add("FlywheelIsAtSetpoint", false)
                .getEntry();
        this.rightEncoderReadingNT = Shuffleboard.getTab("Shooter")
                .add("FlywheelRightEncoderReading", 0.0)
                .getEntry();
        this.leftEncoderReadingNT = Shuffleboard.getTab("Shooter")
                .add("FlywheelLeftEncoderReading", 0.0)
                .getEntry();

        this.rightClosedLoopErrorNT = Shuffleboard.getTab("Shooter")
                .add("FlywheelRightClosedLoopError", 0.0)
                .getEntry();
        this.leftClosedLoopErrorNT = Shuffleboard.getTab("Shooter")
                .add("FlywheelLeftClosedLoopError", 0.0)
                .getEntry();

        // Shuffleboard.getTab("Shooter").add("SpinFlywheelForFenderCommand",
        //         new SpinFlywheelCommand(this, FENDER_VELOCITY));
        // Shuffleboard.getTab("Shooter").add("StopFlywheelCommand", new InstantCommand(this::stopFlywheel, this));

        // If you are re#ding this y#u are in to de#p, get o#t n#w wh#l# y## st##l c#n. ### ###### ### ###### ### #### ###
      
        // Each robot feature that requires PID tuniing has its own Shuffleboard tab for tuning (i.e., "ShooterTuning")
        //  Add indicators and controls to this Shuffleboard tab to assist with interactively tuning the system.

        this.velocitySetPointNT = Shuffleboard.getTab("ShooterTuning")
                .add("VelocitySetpoint", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 25000)) // specify widget properties here
                .getEntry();

        this.motorPowerNT = Shuffleboard.getTab("ShooterTuning")
                .add("Flywheel Power", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.FConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Flywheel F", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.PConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Flywheel P", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.IConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Flywheel I", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.DConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Flywheel D", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();
        
        
    

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.isAtSetpointNT.setBoolean(this.isAtSetpoint());
        this.rightEncoderReadingNT.setDouble(this.rightFlywheelMotor.getSelectedSensorVelocity(FlywheelConstants.SLOT_INDEX));
        this.leftEncoderReadingNT.setDouble(this.leftFlywheelMotor.getSelectedSensorVelocity(FlywheelConstants.SLOT_INDEX));
        this.rightClosedLoopErrorNT.setDouble(this.rightFlywheelMotor.getClosedLoopError(FlywheelConstants.SLOT_INDEX));
        this.leftClosedLoopErrorNT.setDouble(this.leftFlywheelMotor.getClosedLoopError(FlywheelConstants.SLOT_INDEX));

        // the following code will only run when we are tuning the system (i.e., not under normal robot operation)
        if (Constants.TUNING) {

            // when tuning, we first set motor power and check the resulting velocity
            // once we have determined our feedforward constant, comment the following lines
            // and uncomment the ones to tune the PID
            double motorPower = this.motorPowerNT.getDouble(0.0);
            leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, motorPower);
            rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, motorPower);

            // uncomment these lines after dtermining the feedforward
            // this.talonRight.config_kF(SLOT_INDEX, this.FConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonRight.config_kP(SLOT_INDEX, this.PConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonRight.config_kI(SLOT_INDEX, this.IConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonRight.config_kD(SLOT_INDEX, this.DConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonLeft.config_kF(SLOT_INDEX, this.FConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonLeft.config_kP(SLOT_INDEX, this.PConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonLeft.config_kI(SLOT_INDEX, this.IConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);
            // this.talonLeft.config_kD(SLOT_INDEX, this.DConstantNT.getDouble(0.0),
            //         TIMEOUT_MS);

            // double targetVelocity = this.velocitySetPointNT.getDouble(0.0);
            // this.setVelocity(targetVelocity);

        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double getVelocity() {
        return this.leftFlywheelMotor.getSelectedSensorVelocity(FlywheelConstants.SLOT_INDEX);
    }

    public void setVelocity(double velocitySetPoint) {
        this.velocitySetPoint = velocitySetPoint;

        leftFlywheelMotor.set(TalonFXControlMode.Velocity, velocitySetPoint);
        double power = leftFlywheelMotor.get();
        rightFlywheelMotor.set(power);
    }

    public boolean isAtSetpoint() {
        return Math.abs(this.getVelocity() - this.velocitySetPoint) < FlywheelConstants.VELOCITY_TOLERANCE;
    }

    public void stopFlywheel() {
        leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
/*
    public double getIdealLimelight(){
        return limelight.getIdealVelocity();
    }
    */

    
}

