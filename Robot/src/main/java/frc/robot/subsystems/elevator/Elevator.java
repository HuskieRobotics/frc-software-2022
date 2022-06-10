package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExtendClimberToMidRungCommand;
import frc.robot.commands.RetractClimberFullCommand;
import frc.robot.commands.RetractClimberMinimumCommand;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's elevator mechanism. It consists of two motors, which both
 * control the elevator. The right motor is controlled by a PID running on its motor controller to
 * position the elevator at the specified setpoint. The left motor follows the right. It also
 * consists of a Pigeon, which is used to measure the pitch of the robot and determine when to
 * extend and retract the elevator around a rung.
 */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  private double encoderPositionSetpoint;

  private double prevPitch;
  private double[] latestPitches;
  private int latestPitchesIndex;

  private static final String SUBSYSTEM_NAME = "Elevator";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;
  private static final boolean TUNING = false;

  /** Constructs a new Elevator object. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    this.encoderPositionSetpoint = 0.0;

    this.prevPitch = 0.0;
    this.latestPitches = new double[100];
    this.latestPitchesIndex = 0;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    if (DEBUGGING) {
      tab.add("elevator", this);
      tab.addNumber("Encoder", this::getElevatorEncoderHeight);
      tab.addBoolean("Near Local Min?", this::isNearLocalMinimum);
      tab.addBoolean("Near Local Max?", this::isNearLocalMaximum);
      tab.addNumber("Pitch", () -> inputs.pitch);
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
      io.setControlEnabled(true);

      tab.addNumber("Closed Loop Target", this::getSetpoint);
      tab.addNumber("Closed Loop Error", () -> inputs.rightClosedLoopError);
      tab.addNumber("Velocity", () -> inputs.rightVelocity);
      tab.addNumber("Left Motor Volts", () -> inputs.leftAppliedVolts);
      tab.addNumber("Right Motor Volts", () -> inputs.rightAppliedVolts);

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
              event -> io.configureKF(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel P", POSITION_PID_P)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKP(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel I", POSITION_PID_I)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKI(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel D", POSITION_PID_D)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKD(event.getEntry().getValue().getDouble()),
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
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);

    double pitch = inputs.pitch;

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
        io.setMotorPercentage(power);
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
        io.configClosedLoopPeakOutput(POSITION_PID_PEAK_OUTPUT);
      } else {
        io.configClosedLoopPeakOutput(SLOW_PEAK_OUTPUT);
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
          io.setPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_EXTEND);
        }
      } else { // retracting loaded
        // as long as the setpoints are correct, this check is not required as the elevator
        //  will not hit the hardstops
        if (this.getElevatorEncoderHeight() < MIN_ELEVATOR_ENCODER_HEIGHT + 2500) {
          this.stopElevator();
        } else {
          io.setPosition(desiredEncoderPosition, ARBITRARY_FEED_FORWARD_RETRACT);
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
    io.setMotorPercentage(0.0);
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
    io.setControlEnabled(true);
  }

  /**
   * Disables operation of the elevator. Control of the elevator is locked out until enabled with
   * the press of a climb-enable button on the operator console. This is critical because, if the
   * elevator is inadvertently extended when not in the hanger, it may violate the height limit and
   * result in a penalty.
   */
  public void disableElevatorControl() {
    io.setControlEnabled(false);
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
    return inputs.isControlEnabled;
  }

  private double getElevatorEncoderHeight() {
    return inputs.rightPosition;
  }

  private double getSetpoint() {
    return encoderPositionSetpoint;
  }
}
