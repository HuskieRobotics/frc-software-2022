
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExtendClimberToMidRungCommand;
import frc.robot.commands.RetractClimberFullCommand;
import frc.robot.commands.RetractClimberMinimumCommand;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.util.Map;

/**
 *
 */
public class Elevator extends SubsystemBase {
    private NetworkTableEntry elevatorMotorPowerNT;
    private NetworkTableEntry positionSetPointNT;
    private NetworkTableEntry FConstantNT;
    private NetworkTableEntry PConstantNT;
    private NetworkTableEntry IConstantNT;
    private NetworkTableEntry DConstantNT;
    private NetworkTableEntry sCurveConstantNT;
    private NetworkTableEntry velocityConstantNT;
    private NetworkTableEntry accelerationConstantNT;
    private WPI_TalonFX leftElevatorMotor;
    private WPI_TalonFX rightElevatorMotor;
    private final Pigeon2 m_pigeon = new Pigeon2(PIGEON_ID);
    private double encoderPositionSetpoint;
    private boolean isElevatorControlEnabled;
    private double pitchRunningAverage;
    private int pitchRunningAverageSamples;
    private double prevPitch;
    private double[] latestPitches;
    private int latestPitchesIndex;
    private int SAMPLE_WINDOW_WIDTH = 6;   // FIXME: make a constant after tuning
    private double EPSILON = 0.001; // FIXME: make a constant after tuning

    public Elevator() {

        this.latestPitches = new double[100];

        this.leftElevatorMotor = new WPI_TalonFX(LEFT_ELEVATOR_MOTOR_CAN_ID);
        this.rightElevatorMotor = new WPI_TalonFX(RIGHT_ELEVATOR_MOTOR_CAN_ID);

        this.isElevatorControlEnabled = false;
        /* Factory Default all hardware to prevent unexpected behaviour */
        this.rightElevatorMotor.configFactoryDefault();
        this.leftElevatorMotor.configFactoryDefault();

        /** Config Objects for motor controllers */
        TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

        /* Disable all motors */
        this.rightElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.leftElevatorMotor.set(TalonFXControlMode.PercentOutput, 0);

        /* Set neutral modes */
        this.leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
        this.rightElevatorMotor.setNeutralMode(NeutralMode.Brake);

        this.leftElevatorMotor.follow(this.rightElevatorMotor);

        /* Configure output */
        this.rightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);
        this.leftElevatorMotor.setInverted(TalonFXInvertType.FollowMaster);

        /*
         * Talon FX does not need sensor phase set for its integrated sensor
         * This is because it will always be correct if the selected feedback device is
         * integrated sensor (default value)
         * and the user calls getSelectedSensor* to get the sensor's position/velocity.
         * 
         * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
         * sensor-phase
         */
        // this.leftElevatorMotor.setSensorPhase(true);
        // this.rightElevatorMotor.setSensorPhase(true);

        /** Feedback Sensor Configuration */

        /** Distance Configs */

        /* Configure the left Talon's selected sensor as integrated sensor */
        _rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
                                                                                                                    // Feedback
                                                                                                                    // Source

        /* FPID for Distance */
        _rightConfig.slot0.kF = GAINS_POSITION.kF;
        _rightConfig.slot0.kP = GAINS_POSITION.kP;
        _rightConfig.slot0.kI = GAINS_POSITION.kI;
        _rightConfig.slot0.kD = GAINS_POSITION.kD;
        _rightConfig.slot0.integralZone = GAINS_POSITION.kIzone;
        _rightConfig.slot0.closedLoopPeakOutput = GAINS_POSITION.kPeakOutput;

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

        /* Motion Magic Configs */
        _rightConfig.motionAcceleration = ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
        _rightConfig.motionCruiseVelocity = MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
        _rightConfig.motionCurveStrength = SCURVE_STRENGTH;

        /* APPLY the config settings */
        this.rightElevatorMotor.configAllSettings(_rightConfig);

        // this.rightElevatorMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

        /* Initialize */
        this.rightElevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);

		this.leftElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, kTimeoutMs);
        this.leftElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, kTimeoutMs);
        Shuffleboard.getTab("Elevator").addNumber("Encoder Value", this::getElevatorEncoderHeight);
        if(COMMAND_LOGGING) {
            Shuffleboard.getTab("Elevator").add("elevator", this);
            Shuffleboard.getTab("Elevator").addBoolean("Elevator At Setpoint", this::atSetpoint);
            Shuffleboard.getTab("Elevator").addBoolean("Transfer to Secondary", this::hasTransferredToSecondary);
            Shuffleboard.getTab("Elevator").addBoolean("Approaching Next Rung", this::isApproachingNextRung);
            Shuffleboard.getTab("Elevator").addBoolean("Above Next Rung", this::isAboveNextRung);
            Shuffleboard.getTab("Elevator").addBoolean("Below Next Rung", this::isBelowNextRung);
            Shuffleboard.getTab("Elevator").addNumber("Pitch Value", m_pigeon::getPitch);
            Shuffleboard.getTab("Elevator").addNumber("Running Average", this::getPitchRunningAverage);
            //Shuffleboard.getTab("Elevator").addNumber("Encoder Value", this::getElevatorEncoderHeight); 
            Shuffleboard.getTab("Elevator").addBoolean("Near Local Min", this::isNearLocalMinimum);
            Shuffleboard.getTab("Elevator").addBoolean("Near Local Max", this::isNearLocalMaximum);
            //Shuffleboard.getTab("Elevator").addNumber("Closed Loop Target", this::getSetpoint);
            //Shuffleboard.getTab("Elevator").addNumber("Closed Loop Error", this.rightElevatorMotor::getClosedLoopError);
            //Shuffleboard.getTab("Elevator").addNumber("Velocity", this.rightElevatorMotor::getSelectedSensorVelocity);
            //Shuffleboard.getTab("Elevator").addNumber("Left Motor Power", this.leftElevatorMotor::getMotorOutputPercent);
            //Shuffleboard.getTab("Elevator").addNumber("Right Motor Power", this.rightElevatorMotor::getMotorOutputPercent);
            Shuffleboard.getTab("Elevator").add("Extend Climber to Mid", new ExtendClimberToMidRungCommand(this));
            Shuffleboard.getTab("Elevator").add("Retract Climber Full", new RetractClimberFullCommand(this));
            Shuffleboard.getTab("Elevator").add("Retract Climber Minimum", new RetractClimberMinimumCommand(this));
            Shuffleboard.getTab("Elevator").addBoolean("isElevatorControl Enabled", this :: isElevatorControlEnabled);
            
        }

        /* Shuffleboard.getTab("Elevator")
                    .add("sample window", this.SAMPLE_WINDOW_WIDTH)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 50)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.SAMPLE_WINDOW_WIDTH = (int)(event.getEntry().getValue().getDouble());
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        */         
        if (TUNING) {
            this.isElevatorControlEnabled = true;

            this.elevatorMotorPowerNT = Shuffleboard.getTab("Elevator")
                    .add("Elevator Motors", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -1, "max", 1)) // FIX_ME figure max motor power should be 1
                    .getEntry();

            this.positionSetPointNT = Shuffleboard.getTab("Elevator")
                    .add("Position Setpoint", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", MAX_ELEVATOR_HEIGHT))
                    .getEntry();
            this.positionSetPointNT.addListener(event -> {
                this.setElevatorMotorPosition(event.getEntry().getValue().getDouble(), true);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.FConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Flywheel F", GAINS_POSITION.kF)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.FConstantNT.addListener(event -> {
                this.rightElevatorMotor.config_kF(kSlotIdx, event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.PConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Flywheel P", GAINS_POSITION.kP)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.PConstantNT.addListener(event -> {
                this.rightElevatorMotor.config_kP(kSlotIdx, event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.IConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Flywheel I", GAINS_POSITION.kI)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.IConstantNT.addListener(event -> {
                this.rightElevatorMotor.config_kI(kSlotIdx, event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.DConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Flywheel D", GAINS_POSITION.kD)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                    .getEntry();
            this.DConstantNT.addListener(event -> {
                this.rightElevatorMotor.config_kD(kSlotIdx, event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.sCurveConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Scurve Strength", SCURVE_STRENGTH)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 8.0)) // specify widget properties here
                    .getEntry();
            this.sCurveConstantNT.addListener(event -> {
                this.rightElevatorMotor.configMotionSCurveStrength((int) event.getEntry().getValue().getDouble(),
                        kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.velocityConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Max Velocity", MAX_ELEVATOR_VELOCITY)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 8000.0)) // specify widget properties here
                    .getEntry();
            this.velocityConstantNT.addListener(event -> {
                this.rightElevatorMotor.configMotionCruiseVelocity(event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

            this.accelerationConstantNT = Shuffleboard.getTab("Elevator")
                    .add("Max Acceleration", ELEVATOR_ACCELERATION)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", 0, "max", 8000.0)) // specify widget properties here
                    .getEntry();
            this.accelerationConstantNT.addListener(event -> {
                this.rightElevatorMotor.configMotionAcceleration(event.getEntry().getValue().getDouble(), kTimeoutMs);
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

        this.encoderPositionSetpoint = 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double pitch = m_pigeon.getPitch();
        
        // keep the last 100 unique pitches (2 seconds of data)
        if(pitch != this.prevPitch) {
            this.prevPitch = pitch;
            this.latestPitches[this.latestPitchesIndex] = pitch;
            this.latestPitchesIndex++;
            this.latestPitchesIndex %= this.latestPitches.length;
        }

        // keep a running average while approaching the next rung
        double height = this.getElevatorEncoderHeight();
        if(height > TRANSFER_TO_SECONDARY_HEIGHT && height < REACH_JUST_BEFORE_NEXT_RUNG) {
            double average = this.pitchRunningAverage * this.pitchRunningAverageSamples;
            average += pitch;
            this.pitchRunningAverageSamples++;
            this.pitchRunningAverage = average / this.pitchRunningAverageSamples;
        }
        else {
            this.resetPitchRunningAverage();
        }
        
        if (TUNING) {
            this.isElevatorControlEnabled = true;
            // // when tuning, we first set motor power and check the resulting velocity
            // // once we have determined our feedforward constant, comment the following
            // lines
            // and uncomment the ones to tune the PID
            double motorPower = this.elevatorMotorPowerNT.getDouble(0.0);
            this.setElevatorMotorPower(motorPower);

        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public double getPitchRunningAverage() {
        return this.pitchRunningAverage;
    }

    public void resetPitchRunningAverage() {
        this.pitchRunningAverage = 0.0;
        this.pitchRunningAverageSamples = 0;
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double getElevatorEncoderHeight() {
        return this.rightElevatorMotor.getSelectedSensorPosition();
    }

    public void setElevatorMotorPower(double power) {

        if (isElevatorControlEnabled()) {
            if ((power > 0 && this.getElevatorEncoderHeight() > MAX_ELEVATOR_HEIGHT - 5000) ||
                    (power < 0 && this.getElevatorEncoderHeight() < MIN_ELEVATOR_ENCODER_HEIGHT + 5000)) {
                this.stopElevator();
            } else {
                this.leftElevatorMotor.set(ControlMode.PercentOutput, power);
                this.rightElevatorMotor.set(ControlMode.PercentOutput, power);
            }
        }
    }

    public void setElevatorSetpoint(double setpoint) {
        this.encoderPositionSetpoint = setpoint;
    }

    public void setElevatorMotorPosition(double desiredEncoderPosition, boolean isFast) {
        if (isElevatorControlEnabled()) {

            if(isFast) {
                this.rightElevatorMotor.configClosedLoopPeakOutput(kSlotIdx, GAINS_POSITION.kPeakOutput);
            }
            else {
                this.rightElevatorMotor.configClosedLoopPeakOutput(kSlotIdx, SLOW_PEAK_OUTPUT);
            }

            // the feedforward term will be different depending if the elevator is going up
            // or down
            // and if it under load or not; use the desiredEncoderPosition to determine the
            // corresponding feed forward term
            if (desiredEncoderPosition > this.getElevatorEncoderHeight()) { // extending unloaded
                if (this.getElevatorEncoderHeight() > MAX_ELEVATOR_HEIGHT - 2500) {
                    this.stopElevator();
                } else {
                    this.leftElevatorMotor.follow(this.rightElevatorMotor);
                    rightElevatorMotor.set(TalonFXControlMode.Position, desiredEncoderPosition,
                            DemandType.ArbitraryFeedForward, ARBITRARY_FEED_FORWARD_EXTEND);
                }
            } else { // retracting loaded
                if (this.getElevatorEncoderHeight() < MIN_ELEVATOR_ENCODER_HEIGHT + 2500) {
                    this.stopElevator();
                } else {
                    this.leftElevatorMotor.follow(this.rightElevatorMotor);
                    rightElevatorMotor.set(TalonFXControlMode.Position, desiredEncoderPosition,
                            DemandType.ArbitraryFeedForward, ARBITRARY_FEED_FORWARD_RETRACT);
                }
            }

            this.encoderPositionSetpoint = desiredEncoderPosition;
        }
    }

    public boolean atSetpoint() {
        return Math.abs(this.getElevatorEncoderHeight() - this.encoderPositionSetpoint) < ELEVATOR_POSITION_TOLERANCE;
    }

    public void stopElevator() {
        this.leftElevatorMotor.set(ControlMode.PercentOutput, 0.0);
        this.rightElevatorMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean isBelowNextRung() {
        double pitch =  m_pigeon.getPitch();
        if(pitch < this.getPitchRunningAverage()) {
            return true;
        }

        return false;
    }

    public boolean isAboveNextRung() {
        double pitch =  m_pigeon.getPitch();
        if(pitch > this.getPitchRunningAverage()) {
            return true;
        }

        return false;
    }

    public boolean isNearLocalMinimum() {
        // check for a local minimum 2/3 through the sample window
        //  (latestPitchesIndex is the index where the *next* pitch will be stored)
        int potentialLocalMinIndex = (this.latestPitchesIndex - 1) - (SAMPLE_WINDOW_WIDTH / 3);

        // there is a potential to end up with a negative index; wrap around as necessary
        potentialLocalMinIndex = (potentialLocalMinIndex + this.latestPitches.length) % this.latestPitches.length;

        // check if all of the samples before and after the potential local min are greater than the potential local min

        double potentialLocalMin = this.latestPitches[potentialLocalMinIndex];

        int minIndex = this.latestPitchesIndex - SAMPLE_WINDOW_WIDTH;
        minIndex = (minIndex + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

        boolean isLocalMin = true;
        for(int i = 0; i < SAMPLE_WINDOW_WIDTH; i++) {
            int index = minIndex + i;
            index = (index + this.latestPitches.length) % this.latestPitches.length; // handle wrap around
            
            if(this.latestPitches[index] + EPSILON < potentialLocalMin) {
                isLocalMin = false;
            }
        }

        return isLocalMin;
    }

    public boolean isNearLocalMaximum() {
        // check for a local minimum 2/3 through the sample window
        //  (latestPitchesIndex is the index where the *next* pitch will be stored)
        int potentialLocalMaxIndex = (this.latestPitchesIndex - 1) - (SAMPLE_WINDOW_WIDTH / 3);

        // there is a potential to end up with a negative index; wrap around as necessary
        potentialLocalMaxIndex = (potentialLocalMaxIndex + this.latestPitches.length) % this.latestPitches.length;

        // check if all of the samples before and after the potential local min are greater than the potential local min

        double potentialLocalMax = this.latestPitches[potentialLocalMaxIndex];

        int minIndex = this.latestPitchesIndex - SAMPLE_WINDOW_WIDTH;
        minIndex = (minIndex + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

        boolean isLocalMax = true;
        for(int i = 0; i < SAMPLE_WINDOW_WIDTH; i++) {
            int index = minIndex + i;
            index = (index + this.latestPitches.length) % this.latestPitches.length; // handle wrap around
            
            if(this.latestPitches[index] - EPSILON > potentialLocalMax) {
                isLocalMax = false;
            }
        }

        return isLocalMax;

    }

    public boolean hasTransferredToSecondary() {
        return this.getElevatorEncoderHeight() > TRANSFER_TO_SECONDARY_HEIGHT;
    }

    public boolean isApproachingNextRung() {
        return this.getElevatorEncoderHeight() > REACH_JUST_BEFORE_NEXT_RUNG;
    }

    public void elevatorPause(boolean isStartPressed) {
        if (isStartPressed) {
            this.stopElevator();
        }
    }

    public void enableElevatorControl() {
        this.isElevatorControlEnabled = true;
    }

    public void disableElevatorControl() {
        this.isElevatorControlEnabled = false;
    }

    public boolean isElevatorControlEnabled() {
        return this.isElevatorControlEnabled;
    }

    public double getSetpoint() {
        return encoderPositionSetpoint;
    }
}