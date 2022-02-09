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

import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.*;
import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Hood extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX talonSRX3;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private double positionSetPoint;

    private NetworkTableEntry encoderReadingNT;
    private NetworkTableEntry closedLoopErrorNT;
    private NetworkTableEntry positionSetPointNT;
    private NetworkTableEntry PConstantNT;
    private NetworkTableEntry IConstantNT;
    private NetworkTableEntry DConstantNT;

    /**
    *
    */
    public Hood() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        talonSRX3 = new WPI_TalonSRX(3);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        Shuffleboard.getTab("Shooter").add("MoveHoodForFenderCommand",
                new MoveHoodCommand(this, FENDER_POSITION));

        /* Factory Default all hardware to prevent unexpected behaviour */
        this.talonSRX3.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        this.talonSRX3.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        this.talonSRX3.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,
                SLOT_INDEX,
                TIMEOUT_MS);

        /* Config the peak and nominal outputs */
        this.talonSRX3.configNominalOutputForward(0, TIMEOUT_MS);
        this.talonSRX3.configNominalOutputReverse(0, TIMEOUT_MS);
        this.talonSRX3.configPeakOutputForward(1, TIMEOUT_MS);
        this.talonSRX3.configPeakOutputReverse(-1, TIMEOUT_MS);

        /* Config the position closed loop gains in slot0 */
        this.talonSRX3.config_kF(SLOT_INDEX, POSITION_GAINS.kF,
                TIMEOUT_MS);
        this.talonSRX3.config_kP(SLOT_INDEX, POSITION_GAINS.kP,
                TIMEOUT_MS);
        this.talonSRX3.config_kI(SLOT_INDEX, POSITION_GAINS.kI,
                TIMEOUT_MS);
        this.talonSRX3.config_kD(SLOT_INDEX, POSITION_GAINS.kD,
                TIMEOUT_MS);

        /* VERIFY */
        this.talonSRX3.setSensorPhase(true);

        this.positionSetPoint = 0.0;

        this.encoderReadingNT = Shuffleboard.getTab("Shooter")
                .add("HoodEncoderReading", 0.0)
                .getEntry();

        this.closedLoopErrorNT = Shuffleboard.getTab("Shooter")
                .add("HoodClosedLoopError", 0.0)
                .getEntry();

        this.positionSetPointNT = Shuffleboard.getTab("ShooterTuning")
                .add("PositionSetpoint", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 10000)) // specify widget properties here
                .getEntry();

        this.PConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Hood P", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.IConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Hood I", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.DConstantNT = Shuffleboard.getTab("ShooterTuning")
                .add("Hood D", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.encoderReadingNT.setDouble(this.talonSRX3.getSelectedSensorPosition(SLOT_INDEX));
        this.closedLoopErrorNT.setDouble(this.talonSRX3.getClosedLoopError(SLOT_INDEX));

        if (TUNING) {
            this.talonSRX3.config_kF(SLOT_INDEX, 0.0,
                    TIMEOUT_MS);
            this.talonSRX3.config_kP(SLOT_INDEX, this.PConstantNT.getDouble(0.0),
                    TIMEOUT_MS);
            this.talonSRX3.config_kI(SLOT_INDEX, this.IConstantNT.getDouble(0.0),
                    TIMEOUT_MS);
            this.talonSRX3.config_kD(SLOT_INDEX, this.DConstantNT.getDouble(0.0),
                    TIMEOUT_MS);

            double targetPosition = this.positionSetPointNT.getDouble(0.0);
            this.setPosition(targetPosition);

        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public double getPosition() {
        return this.talonSRX3.getSelectedSensorPosition(SLOT_INDEX);
    }

    public void setPosition(double positionSetPoint) {
        this.positionSetPoint = positionSetPoint;
        this.talonSRX3.set(TalonSRXControlMode.Position, this.positionSetPoint);
    }

    public boolean isAtSetpoint() {
        return Math.abs(this.getPosition() - this.positionSetPoint) < POSITION_TOLERANCE;
    }

    public void stopHood() {
        this.talonSRX3.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
}
