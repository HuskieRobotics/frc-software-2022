package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FlywheelConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetFlywheelVelocityCommand;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 *
 */
public class Flywheel extends SubsystemBase {
    private WPI_TalonFX leftFlywheelMotor;
    private WPI_TalonFX rightFlywheelMotor;
    private double velocitySetPoint;
    private int setPointCount;
    private double minVelocityAfterShot;

    /**
    *
    */
    public Flywheel() {
        leftFlywheelMotor = new WPI_TalonFX(LEFT_FLYWHEELMOTOR_CANID);
        rightFlywheelMotor = new WPI_TalonFX(RIGHT_FLYWHEELMOTOR_CANID);

        /* Factory Default all hardware to prevent unexpected behaviour */
        rightFlywheelMotor.configFactoryDefault();
        leftFlywheelMotor.configFactoryDefault();

        /* Config sensor used for Primary PID [Velocity] */
        TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

        /* Disable all motors */
        this.rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);

        /* Set neutral modes */
        this.leftFlywheelMotor.setNeutralMode(NeutralMode.Coast);
        this.rightFlywheelMotor.setNeutralMode(NeutralMode.Coast);

        this.leftFlywheelMotor.follow(this.rightFlywheelMotor);

        /* Configure output */
        this.rightFlywheelMotor.setInverted(TalonFXInvertType.CounterClockwise);
        this.leftFlywheelMotor.setInverted(TalonFXInvertType.Clockwise);

        /*
         * Talon FX does not need sensor phase set for its integrated sensor
         * This is because it will always be correct if the selected feedback device is
         * integrated sensor (default value)
         * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         * 
         * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
         * sensor-phase
         */
        // this.rightFlywheelMotor.setSensorPhase(true);

        /** Feedback Sensor Configuration */

        /** Distance Configs */

        /* Configure the left Talon's selected sensor as integrated sensor */
        _rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
                                                                                                                    // Feedback
                                                                                                                    // Source

        /* FPID for velocity */
        _rightConfig.slot0.kF = GAINS_VELOCITY.kF;
        _rightConfig.slot0.kP = GAINS_VELOCITY.kP;
        _rightConfig.slot0.kI = GAINS_VELOCITY.kI;
        _rightConfig.slot0.kD = GAINS_VELOCITY.kD;
        _rightConfig.slot0.integralZone = GAINS_VELOCITY.kIzone;
        _rightConfig.slot0.closedLoopPeakOutput = GAINS_VELOCITY.kPeakOutput;

        /* Config the neutral deadband. */
        _rightConfig.neutralDeadband = 0.001;

        /**
         * 1ms per loop. PID loop can be slowed down if need be.
         * For example,
         * - if sensor updates are too slow
         * - sensor deltas are very small per update, so derivative error never gets
         * large enough to be useful.
         * - sensor movement is very slow causing the derivative error to be near zero.
         */
        int closedLoopTimeMs = 1;
        _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
        _rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
        _rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
        _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        /* APPLY the config settings */
        this.rightFlywheelMotor.configAllSettings(_rightConfig);

        // rightFlywheelMotor.selectProfileSlot(SLOT_INDEX, PID_LOOP_INDEX);

        this.velocitySetPoint = 0.0;

        Shuffleboard.getTab("MAIN").addBoolean("FlywheelIsAtSetpoint", this::isAtSetpoint);
        
        if(COMMAND_LOGGING) {
            Shuffleboard.getTab("Shooter").add("shooter", this);
            Shuffleboard.getTab("Shooter").addNumber("FlywheelVelocity", this::getVelocity);
            Shuffleboard.getTab("Shooter").addNumber("FlywheelMinVelocity", this::getMinVelocity);
            Shuffleboard.getTab("Shooter").addNumber("FlywheelRightEncoderReading",
                    this.rightFlywheelMotor::getSelectedSensorVelocity);
            Shuffleboard.getTab("Shooter").addNumber("FlywheelLeftEncoderReading",
                    this.leftFlywheelMotor::getSelectedSensorVelocity);
            Shuffleboard.getTab("Shooter").addNumber("FlywheelRightClosedLoopError",
                    this.rightFlywheelMotor::getClosedLoopError);
            Shuffleboard.getTab("Shooter").addNumber("Left Power", this.leftFlywheelMotor::getMotorOutputPercent);
            Shuffleboard.getTab("Shooter").addNumber("Right Power", this.rightFlywheelMotor::getMotorOutputPercent);

            Shuffleboard.getTab("Shooter").add("Wall Shot", new SetFlywheelVelocityCommand(this, WALL_SHOT_VELOCITY));
            Shuffleboard.getTab("Shooter").add("Fender Shot", new SetFlywheelVelocityCommand(this, FENDER_SHOT_VELOCITY));
            Shuffleboard.getTab("Shooter").add("Stop Flywheel", new SetFlywheelVelocityCommand(this, 0));
        }

        // Shuffleboard.getTab("Shooter").add("SpinFlywheelForFenderCommand",
        // new SpinFlywheelCommand(this, FENDER_VELOCITY));
        // Shuffleboard.getTab("Shooter").add("StopFlywheelCommand", new
        // InstantCommand(this::stopFlywheel, this));

        if (TUNING) {
            // Each robot feature that requires PID tuning has its own Shuffleboard tab for
            // tuning (i.e., "Shooter")
            // Add indicators and controls to this Shuffleboard tab to assist with
            // interactively tuning the system.

            Shuffleboard.getTab("Shooter")
                    .add("VelocitySetpoint", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 25000)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.setVelocity(event.getEntry().getValue().getDouble());
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            Shuffleboard.getTab("Shooter")
                    .add("Flywheel Power", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        leftFlywheelMotor.set(TalonFXControlMode.PercentOutput,
                                event.getEntry().getValue().getDouble());
                        rightFlywheelMotor.set(TalonFXControlMode.PercentOutput,
                                event.getEntry().getValue().getDouble());
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            Shuffleboard.getTab("Shooter")
                    .add("Flywheel F", GAINS_VELOCITY.kF)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.rightFlywheelMotor.config_kF(SLOT_INDEX, event.getEntry().getValue().getDouble(),
                                TIMEOUT_MS);
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            Shuffleboard.getTab("Shooter")
                    .add("Flywheel P", GAINS_VELOCITY.kP)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 2.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.rightFlywheelMotor.config_kP(SLOT_INDEX, event.getEntry().getValue().getDouble(),
                                TIMEOUT_MS);
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            Shuffleboard.getTab("Shooter")
                    .add("Flywheel I", GAINS_VELOCITY.kI)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.rightFlywheelMotor.config_kI(SLOT_INDEX, event.getEntry().getValue().getDouble(),
                                TIMEOUT_MS);
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            Shuffleboard.getTab("Shooter")
                    .add("Flywheel D", GAINS_VELOCITY.kD)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.rightFlywheelMotor.config_kD(SLOT_INDEX, event.getEntry().getValue().getDouble(),
                                TIMEOUT_MS);
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }
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

    public double getVelocity() {
        return this.rightFlywheelMotor.getSelectedSensorVelocity(SLOT_INDEX);
    }

    public double getMinVelocity() {
        double velocity = getVelocity();
        if(velocity < this.minVelocityAfterShot && rightFlywheelMotor.get() > 0) {
            minVelocityAfterShot = velocity;
        }
        return this.minVelocityAfterShot;
    }

    public void setVelocity(double velocitySetPoint) {
        this.velocitySetPoint = velocitySetPoint;

        this.leftFlywheelMotor.follow(this.rightFlywheelMotor);
        rightFlywheelMotor.set(TalonFXControlMode.Velocity, velocitySetPoint);
    }

    public double getVelocitySetPoint() {
        return this.velocitySetPoint;
    }

    public boolean isAtSetpoint() {
        if(Math.abs(this.getVelocity() - this.velocitySetPoint) < VELOCITY_TOLERANCE) {
            setPointCount++;
            if(setPointCount >= 10) {
                minVelocityAfterShot = this.getVelocity();
                return true;
            }
        }
        else {
            setPointCount = 0;
        }
            return false;
        
    }

    public void stopFlywheel() {
        leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void unjamFlywheel() {
        leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, REVERSE_POWER);
        rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, REVERSE_POWER);
    }

}
