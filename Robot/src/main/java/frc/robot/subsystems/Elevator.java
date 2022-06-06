package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExtendClimberToMidRungCommand;
import frc.robot.commands.RetractClimberFullCommand;
import frc.robot.commands.RetractClimberMinimumCommand;
import java.util.Map;

/**
 * This subsystem models the robot's elevator mechanism. It consists of two motors, which both
 * control the elevator. The right motor is controlled by a PID running on its motor controller to
 * position the elevator at the specified setpoint. The left motor follows the right. It also
 * consists of a Pigeon, which is used to measure the pitch of the robot and determine when to
 * extend and retract the elevator around a rung.
 */
public class Elevator extends SubsystemBase {
  private WPI_TalonFX leftElevatorMotor;
  private WPI_TalonFX rightElevatorMotor;
  private Pigeon2 pigeon;

  private double encoderPositionSetpoint;
  private boolean isElevatorControlEnabled;

  private double prevPitch;
  private double[] latestPitches;
  private int latestPitchesIndex;

  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;
  private static final boolean TUNING = false;

  /** Constructs a new Elevator object. */
  public Elevator() {

    this.encoderPositionSetpoint = 0.0;
    this.isElevatorControlEnabled = false;

    this.prevPitch = 0.0;
    this.latestPitches = new double[100];
    this.latestPitchesIndex = 0;

    this.leftElevatorMotor = new WPI_TalonFX(LEFT_ELEVATOR_MOTOR_CAN_ID);
    this.rightElevatorMotor = new WPI_TalonFX(RIGHT_ELEVATOR_MOTOR_CAN_ID);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behaviour */
    this.rightElevatorMotor.configFactoryDefault();
    this.leftElevatorMotor.configFactoryDefault();

    /** Config Objects for motor controllers */
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

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
     *
     * this.leftElevatorMotor.setSensorPhase(true)
     * this.rightElevatorMotor.setSensorPhase(true)
     */

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    rightConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    /* FPID for Distance */
    rightConfig.slot0.kF = POSITION_PID_F;
    rightConfig.slot0.kP = POSITION_PID_P;
    rightConfig.slot0.kI = POSITION_PID_I;
    rightConfig.slot0.kD = POSITION_PID_D;
    rightConfig.slot0.integralZone = POSITION_PID_I_ZONE;
    rightConfig.slot0.closedLoopPeakOutput = POSITION_PID_PEAK_OUTPUT;

    /* Config the neutral deadband. */
    rightConfig.neutralDeadband = 0.001;

    /**
     * 1ms per loop. PID loop can be slowed down if need be. For example, - if sensor updates are
     * too slow - sensor deltas are very small per update, so derivative error never gets large
     * enough to be useful. - sensor movement is very slow causing the derivative error to be near
     * zero.
     */
    int closedLoopTimeMs = 1;
    rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    /* Motion Magic Configs */
    rightConfig.motionAcceleration =
        ELEVATOR_ACCELERATION; // (distance units per 100 ms) per second
    rightConfig.motionCruiseVelocity = MAX_ELEVATOR_VELOCITY; // distance units per 100 ms
    rightConfig.motionCurveStrength = SCURVE_STRENGTH;

    /* APPLY the config settings */
    this.rightElevatorMotor.configAllSettings(rightConfig);

    /* Initialize */
    this.rightElevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT_MS);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.leftElevatorMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.leftElevatorMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.pigeon = new Pigeon2(PIGEON_ID);

    addChild("Elevator Left Motor", this.leftElevatorMotor);
    addChild("Elevator Right Motor", this.rightElevatorMotor);

    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getElevatorEncoderHeight);
      tab.addBoolean("Near Local Min?", this::isNearLocalMinimum);
      tab.addBoolean("Near Local Max?", this::isNearLocalMaximum);
      tab.addNumber("Pitch", pigeon::getPitch);
      tab.addBoolean("At Setpoint?", this::atSetpoint);
      tab.addBoolean("Approaching Next Rung?", this::isApproachingNextRung);
      tab.addBoolean("Control Enabled?", this::isElevatorControlEnabled);
    }

    if (TESTING) {
      tab.add("Extend Climber to Mid", new ExtendClimberToMidRungCommand(this));
      tab.add("Retract Climber Full", new RetractClimberFullCommand(this));
      tab.add(
          "Retract Climber Minimum",
          new RetractClimberMinimumCommand(ElevatorConstants.LATCH_HIGH_RUNG_ENCODER_HEIGHT, this));
    }

    if (TUNING) {
      this.isElevatorControlEnabled = true;

      tab.addNumber("Closed Loop Target", this::getSetpoint);
      tab.addNumber("Closed Loop Error", this.rightElevatorMotor::getClosedLoopError);
      tab.addNumber("Velocity", this.rightElevatorMotor::getSelectedSensorVelocity);
      tab.addNumber("Left Motor Power", this.leftElevatorMotor::getMotorOutputPercent);
      tab.addNumber("Right Motor Power", this.rightElevatorMotor::getMotorOutputPercent);

      tab.add("Elevator Motors", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry()
          .addListener(
              event -> this.setElevatorMotorPower(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Position Setpoint", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", MAX_ELEVATOR_HEIGHT))
          .getEntry()
          .addListener(
              event -> this.setElevatorMotorPosition(event.getEntry().getValue().getDouble(), true),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel F", POSITION_PID_F)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.config_kF(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel P", POSITION_PID_P)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.config_kP(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel I", POSITION_PID_I)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.config_kI(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel D", POSITION_PID_D)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.config_kD(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Scurve Strength", SCURVE_STRENGTH)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 8.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.configMotionSCurveStrength(
                      (int) event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Max Velocity", MAX_ELEVATOR_VELOCITY)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 8000.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.configMotionCruiseVelocity(
                      event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Max Acceleration", ELEVATOR_ACCELERATION)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 8000.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightElevatorMotor.configMotionAcceleration(
                      event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the elevator needs to
   * continually record the robot's pitch in order to identify local maxima and minima.
   */
  @Override
  public void periodic() {
    double pitch = pigeon.getPitch();

    // keep the last 100 unique pitches (2 seconds of data)
    if (pitch != this.prevPitch) {
      this.prevPitch = pitch;
      this.latestPitches[this.latestPitchesIndex] = pitch;
      this.latestPitchesIndex++;
      this.latestPitchesIndex %= this.latestPitches.length;
    }
  }

  /**
   * Sets the power of the motors the raise and lower the elevator. This method is intended to only
   * be invoked for manual control of the elevator. Typically, the setElevatorMotorPosition method
   * is invoked to move the elevator to a specified position.
   *
   * @param power the specified power of the elevator's motors as a percentage of full power [-1.0,
   *     1.0]; positive values raise the elevator
   */
  public void setElevatorMotorPower(double power) {
    // since this method is intended for manual control, ensure that the elevator isn't driven
    //  into the hardstops at the top or bottom
    if (isElevatorControlEnabled()) {
      if ((power > 0 && this.getElevatorEncoderHeight() > MAX_ELEVATOR_HEIGHT - 5000)
          || (power < 0 && this.getElevatorEncoderHeight() < MIN_ELEVATOR_ENCODER_HEIGHT + 5000)) {
        this.stopElevator();
      } else {
        this.leftElevatorMotor.set(ControlMode.PercentOutput, power);
        this.rightElevatorMotor.set(ControlMode.PercentOutput, power);
      }
    }
  }

  /**
   * Sets the setpoint of the elevator to the specified position and moves the elevator towards that
   * position with the power capped at the specified value.
   *
   * @param desiredEncoderPosition the specified position of the elevator (in ticks); ticks are 0
   *     when the elevator is fully retracted
   * @param isFast if true, move the elevator at maximum power; if false, move the elevator slowly
   */
  public void setElevatorMotorPosition(double desiredEncoderPosition, boolean isFast) {
    // Control of the elevator is locked out until enabled with the press of a climb-enable
    //  button on the operator console. This is critical because, if the elevator is
    //  inadvertently extended when not in the hanger, it may violate the height limit and
    //  result in a penalty.
    if (isElevatorControlEnabled()) {

      if (isFast) {
        this.rightElevatorMotor.configClosedLoopPeakOutput(SLOT_INDEX, POSITION_PID_PEAK_OUTPUT);
      } else {
        this.rightElevatorMotor.configClosedLoopPeakOutput(SLOT_INDEX, SLOW_PEAK_OUTPUT);
      }

      // the feedforward term will be different depending if the elevator is going up
      // or down and if it under load or not; use the desiredEncoderPosition to determine the
      // corresponding feed forward term
      if (desiredEncoderPosition > this.getElevatorEncoderHeight()) { // extending unloaded
        // as long as the setpoints are correct, this check is not required as the elevator
        //  will not hit the hardstops
        if (this.getElevatorEncoderHeight() > MAX_ELEVATOR_HEIGHT - 2500) {
          this.stopElevator();
        } else {
          this.leftElevatorMotor.follow(this.rightElevatorMotor);
          rightElevatorMotor.set(
              TalonFXControlMode.Position,
              desiredEncoderPosition,
              DemandType.ArbitraryFeedForward,
              ARBITRARY_FEED_FORWARD_EXTEND);
        }
      } else { // retracting loaded
        // as long as the setpoints are correct, this check is not required as the elevator
        //  will not hit the hardstops
        if (this.getElevatorEncoderHeight() < MIN_ELEVATOR_ENCODER_HEIGHT + 2500) {
          this.stopElevator();
        } else {
          this.leftElevatorMotor.follow(this.rightElevatorMotor);
          rightElevatorMotor.set(
              TalonFXControlMode.Position,
              desiredEncoderPosition,
              DemandType.ArbitraryFeedForward,
              ARBITRARY_FEED_FORWARD_RETRACT);
        }
      }

      this.encoderPositionSetpoint = desiredEncoderPosition;
    }
  }

  /**
   * Returns true if the elevator's position is at the specified setpoint (i.e., within the desired
   * tolerance) or if elevator control is not enabled. The tolerance is critical since it is highly
   * unlikely that the position of the elevator will match the setpoint exactly. Based on impirical
   * observations, there is little to no overshoot of the setpoint. Therefore, there is no need to
   * wait additional iterations and provide time to settle.
   *
   * @return true if the elevator's position is at the specified setpoint (i.e., within the desired
   *     tolerance) or if elevator control is not enabled.
   */
  public boolean atSetpoint() {
    if (!isElevatorControlEnabled()) {
      return true;
    }

    return Math.abs(this.getElevatorEncoderHeight() - this.encoderPositionSetpoint)
        < ELEVATOR_POSITION_TOLERANCE;
  }

  /**
   * Stops the elevator. Since the elevator's motors are in brake mode, the elevator will stop
   * moving almost immediately after this method is executed.
   */
  public void stopElevator() {
    this.leftElevatorMotor.set(ControlMode.PercentOutput, 0.0);
    this.rightElevatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Returns true if the robot's swing is just beyond a local minimum. This is useful for
   * determining when it is time to extend the elevator below a rung.
   *
   * @return true if the robot's swing is just beyond a local minimum
   */
  public boolean isNearLocalMinimum() {
    // check for a local minimum 2/3 through the sample window
    //  (latestPitchesIndex is the index where the *next* pitch will be stored)
    int potentialLocalMinIndex = (this.latestPitchesIndex - 1) - (SAMPLE_WINDOW_WIDTH / 3);

    // there is a potential to end up with a negative index; wrap around as necessary
    potentialLocalMinIndex =
        (potentialLocalMinIndex + this.latestPitches.length) % this.latestPitches.length;

    // check if all of the samples before and after the potential local min are greater than the
    // potential local min

    double potentialLocalMin = this.latestPitches[potentialLocalMinIndex];

    int minIndex = this.latestPitchesIndex - SAMPLE_WINDOW_WIDTH;
    minIndex =
        (minIndex + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

    boolean isLocalMin = true;
    for (int i = 0; i < SAMPLE_WINDOW_WIDTH; i++) {
      int index = minIndex + i;
      index = (index + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

      if (this.latestPitches[index] + EPSILON < potentialLocalMin) {
        isLocalMin = false;
      }
    }

    return isLocalMin;
  }

  /**
   * Returns true if the robot's swing is just beyond a local maximum. This is useful for
   * determining when it is time to extend the elevator above a rung.
   *
   * @return true if the robot's swing is just beyond a local maximum
   */
  public boolean isNearLocalMaximum() {
    // check for a local minimum 2/3 through the sample window
    //  (latestPitchesIndex is the index where the *next* pitch will be stored)
    int potentialLocalMaxIndex = (this.latestPitchesIndex - 1) - (SAMPLE_WINDOW_WIDTH / 3);

    // there is a potential to end up with a negative index; wrap around as necessary
    potentialLocalMaxIndex =
        (potentialLocalMaxIndex + this.latestPitches.length) % this.latestPitches.length;

    // check if all of the samples before and after the potential local min are greater than the
    // potential local min

    double potentialLocalMax = this.latestPitches[potentialLocalMaxIndex];

    int minIndex = this.latestPitchesIndex - SAMPLE_WINDOW_WIDTH;
    minIndex =
        (minIndex + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

    boolean isLocalMax = true;
    for (int i = 0; i < SAMPLE_WINDOW_WIDTH; i++) {
      int index = minIndex + i;
      index = (index + this.latestPitches.length) % this.latestPitches.length; // handle wrap around

      if (this.latestPitches[index] - EPSILON > potentialLocalMax) {
        isLocalMax = false;
      }
    }

    return isLocalMax;
  }

  /**
   * Returns true if the elevator is approaching the next rung. This is used to determine when it is
   * time to ensure that the elevator passes below the next rung.
   *
   * @return true if the elevator is approaching the next rung
   */
  public boolean isApproachingNextRung() {
    return this.getElevatorEncoderHeight() > REACH_JUST_BEFORE_NEXT_RUNG;
  }

  /**
   * Stops the elevator. This method is intended to only be invoked for manual control of the
   * elevator. To prevent inadvertent operation, this method requires that two buttons are pressed
   * simultaneously to stop the elevator.
   *
   * @param isStartPressed true if the second button is also pressed, which is required to stop the
   *     elevator
   */
  public void elevatorPause(boolean isStartPressed) {
    if (isStartPressed) {
      this.stopElevator();
    }
  }

  /**
   * Enables operation of the elevator. Control of the elevator is locked out until enabled with the
   * press of a climb-enable button on the operator console, which invokes this method. This is
   * critical because, if the elevator is inadvertently extended when not in the hanger, it may
   * violate the height limit and result in a penalty.
   */
  public void enableElevatorControl() {
    this.isElevatorControlEnabled = true;
  }

  /**
   * Disables operation of the elevator. Control of the elevator is locked out until enabled with
   * the press of a climb-enable button on the operator console. This is critical because, if the
   * elevator is inadvertently extended when not in the hanger, it may violate the height limit and
   * result in a penalty.
   */
  public void disableElevatorControl() {
    this.isElevatorControlEnabled = false;
  }

  /**
   * Returns true if control of the elevator is enabled. Control of the elevator is locked out until
   * enabled with the press of a climb-enable button on the operator console. This is critical
   * because, if the elevator is inadvertently extended when not in the hanger, it may violate the
   * height limit and result in a penalty.
   *
   * @return true if control of the elevator is enabled
   */
  public boolean isElevatorControlEnabled() {
    return this.isElevatorControlEnabled;
  }

  private double getElevatorEncoderHeight() {
    return this.rightElevatorMotor.getSelectedSensorPosition();
  }

  private double getSetpoint() {
    return encoderPositionSetpoint;
  }
}
