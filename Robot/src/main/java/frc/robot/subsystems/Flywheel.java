package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetFlywheelVelocityCommand;
import java.util.Map;

/**
 * This subsystem models the robot's flywheel mechanism. It consists of two motors, which both
 * rotate the flywheel. The right motor is controlled by a PID running on its motor controller to
 * keep the flywheel's velocity at the specified setpoint. The left motor follows the right.
 */
public class Flywheel extends SubsystemBase {
  private WPI_TalonFX leftFlywheelMotor;
  private WPI_TalonFX rightFlywheelMotor;
  private double velocitySetPoint;
  private int atSetpointIterationCount;

  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;
  private static final boolean TUNING = false;

  /** Constructs a new Flywheel object. */
  public Flywheel() {
    leftFlywheelMotor = new WPI_TalonFX(LEFT_FLYWHEELMOTOR_CANID);
    rightFlywheelMotor = new WPI_TalonFX(RIGHT_FLYWHEELMOTOR_CANID);

    // the following configuration is based on the CTRE example code

    /* Factory Default all hardware to prevent unexpected behaviour */
    rightFlywheelMotor.configFactoryDefault();
    leftFlywheelMotor.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    /* Disable all motors */
    this.rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);
    this.leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0);

    /* Set neutral modes */
    this.leftFlywheelMotor.setNeutralMode(NeutralMode.Brake);
    this.rightFlywheelMotor.setNeutralMode(NeutralMode.Brake);

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
     *
     * this.rightFlywheelMotor.setSensorPhase(true)
     */

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    rightConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    /* FPID for velocity */
    rightConfig.slot0.kF = VELOCITY_PID_F;
    rightConfig.slot0.kP = VELOCITY_PID_P;
    rightConfig.slot0.kI = VELOCITY_PID_I;
    rightConfig.slot0.kD = VELOCITY_PID_D;
    rightConfig.slot0.integralZone = VELOCITY_PID_I_ZONE;
    rightConfig.slot0.closedLoopPeakOutput = VELOCITY_PID_PEAK_OUTPUT;

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

    /* APPLY the config settings */
    this.rightFlywheelMotor.configAllSettings(rightConfig);

    // these status frames aren't read; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.leftFlywheelMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.leftFlywheelMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.velocitySetPoint = 0.0;

    addChild("Flywheel Left Motor", this.leftFlywheelMotor);
    addChild("Flywheel Right Motor", this.rightFlywheelMotor);

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    if (DEBUGGING) {
      tab.add("Flywheel", this);

      tab.addBoolean("At Setpoint?", this::isAtSetpoint);
      tab.addNumber("Velocity", this::getVelocity);
      tab.addNumber("Right Velocity", this.rightFlywheelMotor::getSelectedSensorVelocity);
      tab.addNumber("Left Velocity", this.leftFlywheelMotor::getSelectedSensorVelocity);
      tab.addNumber("Right Closed Loop Error", this.rightFlywheelMotor::getClosedLoopError);
      tab.addNumber("Left Power", this.leftFlywheelMotor::getMotorOutputPercent);
      tab.addNumber("Right Power", this.rightFlywheelMotor::getMotorOutputPercent);
    }

    if (TESTING) {
      tab.add("Wall Shot", new SetFlywheelVelocityCommand(this, WALL_SHOT_VELOCITY));
      tab.add("Launchpad Shot", new SetFlywheelVelocityCommand(this, LAUNCH_PAD_VELOCITY));
      tab.add("Stop Flywheel", new InstantCommand(this::stopFlywheel, this));
    }

    if (TUNING) {
      tab.add("Velocity Setpoint", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 25000)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> this.setVelocity(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel Power", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> {
                leftFlywheelMotor.set(
                    TalonFXControlMode.PercentOutput, event.getEntry().getValue().getDouble());
                rightFlywheelMotor.set(
                    TalonFXControlMode.PercentOutput, event.getEntry().getValue().getDouble());
              },
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel F", VELOCITY_PID_F)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightFlywheelMotor.config_kF(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel P", VELOCITY_PID_P)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 2.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightFlywheelMotor.config_kP(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel I", VELOCITY_PID_I)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightFlywheelMotor.config_kI(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel D", VELOCITY_PID_D)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event ->
                  this.rightFlywheelMotor.config_kD(
                      SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  private double getVelocity() {
    return this.rightFlywheelMotor.getSelectedSensorVelocity(SLOT_INDEX);
  }

  /**
   * Sets the velocity of the flywheel to the specified value.
   *
   * @param velocitySetPoint the desired velocity of the flywheel in units of ticks per 100 ms
   */
  public void setVelocity(double velocitySetPoint) {
    this.velocitySetPoint = velocitySetPoint;

    this.leftFlywheelMotor.follow(this.rightFlywheelMotor);
    rightFlywheelMotor.set(TalonFXControlMode.Velocity, velocitySetPoint);
  }

  /**
   * Returns true if the flywheel's velocity is at the specified setpoint (i.e., within the desired
   * tolerance for the specified number of iterations). The tolerance is critical since it is highly
   * unlikely that the velocity of the flywheel will match the setpoint exactly. Waiting the
   * specified number of iterations is critical since the PID may overshoot the setpoint and need
   * additional time to settle. The flywheel is only considered at the specified velocity if it
   * remains at that velocity continuously for the desired number of iterations. Without waiting, it
   * would be reported that the flywheel had reached the specified velocity but then, when the cargo
   * is shot, the velocity would be too great.
   *
   * @return true if the flywheel's velocity is at the specified setpoint (i.e., within the desired
   *     tolerance for the specified number of iterations)
   */
  public boolean isAtSetpoint() {
    if (Math.abs(this.getVelocity() - this.velocitySetPoint) < VELOCITY_TOLERANCE) {
      atSetpointIterationCount++;
      if (atSetpointIterationCount >= SETPOINTCOUNT) {
        return true;
      }
    } else {
      atSetpointIterationCount = 0;
    }

    return false;
  }

  /**
   * Stops the flywheel. Since the flywheel's motors are in brake mode, the flywheel will stop
   * spinning shortly after this method is executed.
   */
  public void stopFlywheel() {
    leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  /**
   * Reverses the normal direction of the flywheel at the desired power. This method is invoked if
   * cargo has jammed in the flywheel or storage.
   */
  public void unjamFlywheel() {
    leftFlywheelMotor.set(TalonFXControlMode.PercentOutput, REVERSE_POWER);
    rightFlywheelMotor.set(TalonFXControlMode.PercentOutput, REVERSE_POWER);
  }
}
