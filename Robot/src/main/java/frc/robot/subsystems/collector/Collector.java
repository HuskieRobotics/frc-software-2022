package frc.robot.subsystems.collector;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collector.CollectorIO.CollectorIOInputs;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's collector mechanism. It consists of a single motor, which
 * rotates the collector's intake wheels in an intake or outtake direction, and a solenoid which,
 * when enabled, deploys the collector; and, when disabled, retracts the collector.
 */
public class Collector extends SubsystemBase {
  private final CollectorIO io;
  private final CollectorIOInputs inputs = new CollectorIOInputs();

  private static final String SUBSYSTEM_NAME = "Collector";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new Collector object. */
  public Collector(CollectorIO io) {
    this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    if (DEBUGGING) {
      tab.add(SUBSYSTEM_NAME, this);
      tab.addBoolean("Enabled?", this::isEnabled);
    }

    if (TESTING) {
      tab.add("Collector Speed", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry()
          .addListener(
              event -> io.setMotorPercentage(event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      tab.add("Deploy Collector", new InstantCommand(() -> io.setExtended(true), this));
      tab.add("Retract Collector", new InstantCommand(() -> io.setExtended(false), this));
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

  /**
   * Sets the power of the collector's motor, which rotates the collector's intake wheels, to the
   * specified value.
   *
   * @param power the specified power of the collector's motor as a percentage of full power [-1.0,
   *     1.0]; positive values rotate the wheels in the intake direction
   */
  public void setCollectorPower(double power) {
    io.setMotorPercentage(power);
  }

  /**
   * Disables the collector subsystem. This results in stopping the collector's intake wheels and
   * retracting the collector back inside the robot frame.
   */
  public void disableCollector() {
    io.setEnabled(false);
    io.setMotorPercentage(0.0);
    io.setExtended(false);
  }

  /**
   * Enables the collector subsystem. This results in spinning the collector's intake wheels in the
   * intake direcion and extending the collector outside the robot frame to position it to collect
   * cargo.
   */
  public void enableCollector() {
    io.setEnabled(true);
    io.setMotorPercentage(CollectorConstants.COLLECTOR_DEFAULT_POWER);
    io.setExtended(true);
  }

  /**
   * Returns true if the collector is enabled (i.e., intake wheels spinning and collector deployed).
   *
   * @return true if the collector is enabled (i.e., intake wheels spinning and collector deployed)
   */
  public boolean isEnabled() {
    return inputs.isEnabled;
  }
}
