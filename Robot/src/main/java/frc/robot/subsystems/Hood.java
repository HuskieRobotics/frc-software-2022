package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetHoodPositionCommand;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 *
 */
public class Hood extends SubsystemBase {
    private WPI_TalonSRX hoodMotor;
    private NetworkTableEntry motorPowerNT;
    private NetworkTableEntry FConstantHoodNT;
    private NetworkTableEntry PConstantHoodNT;
    private NetworkTableEntry IConstantHoodNT;
    private NetworkTableEntry DConstantHoodNT;
    private NetworkTableEntry setRotationsNT;
    private double positionSetpoint;

    /**
    *
    */
    public Hood() {

        this.hoodMotor = new WPI_TalonSRX(HOOD_MOTOR_ID);

        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters
         * in the SPARK MAX to their factory default state. If no argument is passed,
         * these
         * parameters will not persist between power cycles
         */
        this.hoodMotor.configFactoryDefault();

        /** Config Objects for motor controllers */
        TalonSRXConfiguration _config = new TalonSRXConfiguration();

        /* Disable all motors */
        this.hoodMotor.set(TalonSRXControlMode.PercentOutput, 0);

        /* Set neutral modes */
        this.hoodMotor.setNeutralMode(NeutralMode.Brake);

        /* Configure output */
        this.hoodMotor.setInverted(InvertType.InvertMotorOutput);
        // this.hoodMotor.setSensorPhase(true);

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController
         * object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        _config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

        /* FPID for Distance */
        _config.slot0.kF = KFF;
        _config.slot0.kP = KP;
        _config.slot0.kI = KI;
        _config.slot0.kD = KD;
        _config.slot0.integralZone = KIz;
        _config.slot0.closedLoopPeakOutput = K_MAX_OUTPUT;

        /* Config the neutral deadband. */
        _config.neutralDeadband = 0.001;

        /**
         * 1ms per loop. PID loop can be slowed down if need be.
         * For example,
         * - if sensor updates are too slow
         * - sensor deltas are very small per update, so derivative error never gets
         * large enough to be useful.
         * - sensor movement is very slow causing the derivative error to be near zero.
         */
        int closedLoopTimeMs = 1;
        _config.slot0.closedLoopPeriod = closedLoopTimeMs;
        _config.slot1.closedLoopPeriod = closedLoopTimeMs;
        _config.slot2.closedLoopPeriod = closedLoopTimeMs;
        _config.slot3.closedLoopPeriod = closedLoopTimeMs;

        /* APPLY the config settings */
        this.hoodMotor.configAllSettings(_config);

        /* Initialize */
        this.hoodMotor.setSelectedSensorPosition(0, PID_SLOT, TIMEOUT_MS);

        if (TUNING) {
            this.motorPowerNT = Shuffleboard.getTab("Hood")
                    .add("motor power", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.motorPowerNT.addListener(event -> {
                this.setHoodMotorPower(event.getEntry().getValue().getDouble());
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.FConstantHoodNT = Shuffleboard.getTab("Hood")
                    .add("kF", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.FConstantHoodNT.addListener(event -> {
                hoodMotor.config_kF(PID_SLOT, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.PConstantHoodNT = Shuffleboard.getTab("Hood")
                    .add("kP", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.PConstantHoodNT.addListener(event -> {
                hoodMotor.config_kP(PID_SLOT, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.IConstantHoodNT = Shuffleboard.getTab("Hood")
                    .add("kI", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.IConstantHoodNT.addListener(event -> {
                hoodMotor.config_kI(PID_SLOT, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.DConstantHoodNT = Shuffleboard.getTab("Hood")
                    .add("kD", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.DConstantHoodNT.addListener(event -> {
                hoodMotor.config_kD(PID_SLOT, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.setRotationsNT = Shuffleboard.getTab("Hood")
                    .add("setRotations", 0.0)
                    .getEntry();
            this.setRotationsNT.addListener(event -> {
                this.setHoodSetpoint(event.getEntry().getValue().getDouble());
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

        Shuffleboard.getTab("Hood").addNumber("HoodEncoderReading", this::getHoodEncoderPosition);
        // Shuffleboard.getTab("Hood").addNumber("HoodLimelightRotations",
        // this::getHoodSetpointLimeLight);
        Shuffleboard.getTab("Hood").addBoolean("IsAtSetpoint", this::isAtSetpoint);
        Shuffleboard.getTab("Hood").addNumber("Current Setpoint", this::getPositionSetpoint);
        Shuffleboard.getTab("Hood").addNumber("Hood Motor Power", hoodMotor::get);

        Shuffleboard.getTab("Hood").add("Aim Low", new SetHoodPositionCommand(this, LOW_ANGLE));
        Shuffleboard.getTab("Hood").add("Aim High", new SetHoodPositionCommand(this, HIGH_ANGLE));

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public double getHoodEncoderPosition() {
        return hoodMotor.getSelectedSensorPosition();
    }

    // public double getHoodSetpointLimeLight() {

    // double a = lm.getIdealHoodA();
    // return a * HOOD_DEGREES_TO_HOOD_ENCODER; //will this return rotations?
    // }

    public void setHoodMotorPower(double pwr) {
        this.hoodMotor.set(ControlMode.PercentOutput, pwr);
    }

    public void aimForFender() {
        this.setHoodSetpoint(LOW_ANGLE);
    }

    public void aimForWall() {
        this.setHoodSetpoint(HIGH_ANGLE);
    }

    public void setHoodSetpoint(double rotations) {

        /**
         * PIDController objects are commanded to a set point using the
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four
         * parameters:
         * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         * com.revrobotics.CANSparkMax.ControlType.kPosition
         * com.revrobotics.CANSparkMax.ControlType.kVelocity
         * com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        this.positionSetpoint = rotations;
        if (rotations > getHoodEncoderPosition()) {
            hoodMotor.set(ControlMode.Position, rotations, DemandType.ArbitraryFeedForward,
                    ARBITRARY_FEED_FORWARD_UP_IN_PERCENT);
        } else {
            hoodMotor.set(ControlMode.Position, rotations, DemandType.ArbitraryFeedForward,
                    ARBITRARY_FEED_FORWARD_DOWN_IN_PERCENT);
        }

    }

    public boolean isAtSetpoint() {

        return Math.abs(this.getHoodEncoderPosition() - this.positionSetpoint) < POSITION_TOLERANCE;
    }

    public double getPositionSetpoint() {
        return this.positionSetpoint;
    }

}
