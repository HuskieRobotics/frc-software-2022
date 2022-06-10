package frc.robot.subsystems.flywheel;

import static frc.robot.Constants.FlywheelConstants.*;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetFlywheelVelocityCommand;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's flywheel mechanism. It consists of two motors, which both
 * rotate the flywheel. The right motor is controlled by a PID running on its motor controller to
 * keep the flywheel's velocity at the specified setpoint. The left motor follows the right.
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();

  private double velocitySetPoint;
  private int atSetpointIterationCount;

  private static final String SUBSYSTEM_NAME = "Shooter";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;
  private static final boolean TUNING = false;

  /** Constructs a new Flywheel object. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    this.velocitySetPoint = 0.0;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    if (DEBUGGING) {
      tab.add(SUBSYSTEM_NAME, this);

      tab.addBoolean("At Setpoint?", this::isAtSetpoint);
      tab.addNumber("Velocity", this::getVelocity);
      tab.addNumber("Right Velocity", () -> inputs.rightVelocity);
      tab.addNumber("Left Velocity", () -> inputs.leftVelocity);
      tab.addNumber("Right Closed Loop Error", () -> inputs.rightClosedLoopError);
      tab.addNumber("Left Voltage", () -> inputs.leftAppliedVolts);
      tab.addNumber("Right Voltage", () -> inputs.rightAppliedVolts);
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
              event -> io.setMotorPercentage(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel F", VELOCITY_PID_F)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKF(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel P", VELOCITY_PID_P)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 2.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKP(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel I", VELOCITY_PID_I)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKI(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      tab.add("Flywheel D", VELOCITY_PID_D)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> io.configureKD(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  /**
   * For each iteration, the subsystem's periodic method is invoked before any commands are
   * executed.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs(SUBSYSTEM_NAME, inputs);
  }

  private double getVelocity() {
    return inputs.rightVelocity;
  }

  /**
   * Sets the velocity of the flywheel to the specified value.
   *
   * @param velocitySetPoint the desired velocity of the flywheel in units of ticks per 100 ms
   */
  public void setVelocity(double velocitySetPoint) {
    this.velocitySetPoint = velocitySetPoint;

    io.setVelocity(velocitySetPoint);
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
    io.setMotorPercentage(0.0);
  }

  /**
   * Reverses the normal direction of the flywheel at the desired power. This method is invoked if
   * cargo has jammed in the flywheel or storage.
   */
  public void unjamFlywheel() {
    io.setMotorPercentage(REVERSE_POWER);
  }
}
