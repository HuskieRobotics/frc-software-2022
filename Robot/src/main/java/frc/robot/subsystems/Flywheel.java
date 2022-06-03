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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private int setPointCount;
  private double minVelocityAfterShot;

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
     */
    // this.rightFlywheelMotor.setSensorPhase(true);

    /** Feedback Sensor Configuration */

    /** Distance Configs */

    /* Configure the left Talon's selected sensor as integrated sensor */
    rightConfig.primaryPID.selectedFeedbackSensor =
        TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
    // Feedback
    // Source

    /* FPID for velocity */
    rightConfig.slot0.kF = GAINS_VELOCITY.kF;
    rightConfig.slot0.kP = GAINS_VELOCITY.kP;
    rightConfig.slot0.kI = GAINS_VELOCITY.kI;
    rightConfig.slot0.kD = GAINS_VELOCITY.kD;
    rightConfig.slot0.integralZone = GAINS_VELOCITY.kIzone;
    rightConfig.slot0.closedLoopPeakOutput = GAINS_VELOCITY.kPeakOutput;

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

    // rightFlywheelMotor.selectProfileSlot(SLOT_INDEX, PID_LOOP_INDEX);

    this.velocitySetPoint = 0.0;

    // Shuffleboard.getTab("MAIN").addBoolean("FlywheelIsAtSetpoint", this::isAtSetpoint);
    if (Constants.COMMAND_LOGGING) {
      Shuffleboard.getTab("Shooter").addBoolean("FlywheelIsAtSetpoint", this::isAtSetpoint);
      Shuffleboard.getTab("MAIN").add("shooter", this);

      Shuffleboard.getTab("Shooter").addNumber("FlywheelVelocity", this::getVelocity);
      Shuffleboard.getTab("Shooter").addNumber("FlywheelMinVelocity", this::getMinVelocity);
      Shuffleboard.getTab("Shooter")
          .addNumber(
              "FlywheelRightEncoderReading", this.rightFlywheelMotor::getSelectedSensorVelocity);
      Shuffleboard.getTab("Shooter")
          .addNumber(
              "FlywheelLeftEncoderReading", this.leftFlywheelMotor::getSelectedSensorVelocity);
      Shuffleboard.getTab("Shooter")
          .addNumber("FlywheelRightClosedLoopError", this.rightFlywheelMotor::getClosedLoopError);
      Shuffleboard.getTab("Shooter")
          .addNumber("Left Power", this.leftFlywheelMotor::getMotorOutputPercent);
      Shuffleboard.getTab("Shooter")
          .addNumber("Right Power", this.rightFlywheelMotor::getMotorOutputPercent);

      Shuffleboard.getTab("Shooter")
          .add("Wall Shot", new SetFlywheelVelocityCommand(this, WALL_SHOT_VELOCITY));
      Shuffleboard.getTab("Shooter")
          .add("Launchpad Shot", new SetFlywheelVelocityCommand(this, LAUNCH_PAD_VELOCITY));
      Shuffleboard.getTab("Shooter")
          .add("Stop Flywheel", new InstantCommand(this::stopFlywheel, this));
    }

    // Shuffleboard.getTab("Shooter").add("SpinFlywheelForFenderCommand",
    // new SpinFlywheelCommand(this, FENDER_VELOCITY));
    // Shuffleboard.getTab("Shooter").add("StopFlywheelCommand", new
    // InstantCommand(this::stopFlywheel, this));

    // if (Constants.TUNING) {
    // Each robot feature that requires PID tuning has its own Shuffleboard tab for
    // tuning (i.e., "Shooter")
    // Add indicators and controls to this Shuffleboard tab to assist with
    // interactively tuning the system.

    Shuffleboard.getTab("Shooter")
        .add("VelocitySetpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 25000)) // specify widget properties here
        .getEntry()
        .addListener(
            event -> {
              this.setVelocity(event.getEntry().getValue().getDouble());
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Shooter")
        .add("Flywheel Power", 0.0)
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

    Shuffleboard.getTab("Shooter")
        .add("Flywheel F", GAINS_VELOCITY.kF)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
        .getEntry()
        .addListener(
            event -> {
              this.rightFlywheelMotor.config_kF(
                  SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Shooter")
        .add("Flywheel P", GAINS_VELOCITY.kP)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2.0)) // specify widget properties here
        .getEntry()
        .addListener(
            event -> {
              this.rightFlywheelMotor.config_kP(
                  SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Shooter")
        .add("Flywheel I", GAINS_VELOCITY.kI)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
        .getEntry()
        .addListener(
            event -> {
              this.rightFlywheelMotor.config_kI(
                  SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    Shuffleboard.getTab("Shooter")
        .add("Flywheel D", GAINS_VELOCITY.kD)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
        .getEntry()
        .addListener(
            event -> {
              this.rightFlywheelMotor.config_kD(
                  SLOT_INDEX, event.getEntry().getValue().getDouble(), TIMEOUT_MS);
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    // }
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

  // make private
  public double getVelocity() {
    return this.rightFlywheelMotor.getSelectedSensorVelocity(SLOT_INDEX);
  }

  // delete
  public double getMinVelocity() {
    double velocity = getVelocity();
    if (velocity < this.minVelocityAfterShot && rightFlywheelMotor.get() > 0) {
      minVelocityAfterShot = velocity;
    }
    return this.minVelocityAfterShot;
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

  // delete
  public double getVelocitySetPoint() {
    return this.velocitySetPoint;
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
      setPointCount++;
      if (setPointCount >= SETPOINTCOUNT) {
        minVelocityAfterShot = this.getVelocity();
        return true;
      }
    } else {
      setPointCount = 0;
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
