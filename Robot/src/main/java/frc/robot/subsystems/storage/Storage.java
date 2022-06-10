package frc.robot.subsystems.storage;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SortStorageCommand;
import frc.robot.subsystems.storage.StorageIO.StorageIOInputs;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's storage mechanism. Is consists of a single motor, which moves
 * the storage's belt in an intake or outtake direction, and two sensors which detect cargo at the
 * collector end and the shooter end of the storage.
 */
public class Storage extends SubsystemBase {
  private final StorageIO io;
  private final StorageIOInputs inputs = new StorageIOInputs();

  private static final String SUBSYSTEM_NAME = "Storage";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new Storage object. */
  public Storage(StorageIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    if (DEBUGGING) {
      tab.add(SUBSYSTEM_NAME, this);
      tab.addBoolean("Collector Unblocked?", this::isCollectorSensorUnblocked);
      tab.addBoolean("Shooter Unblocked?", this::isShooterSensorUnblocked);
    }

    if (TESTING) {
      tab.add("Sort Storage", new SortStorageCommand(this));
      tab.add("Belt Power", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(
              event -> this.setStoragePower(event.getEntry().getValue().getDouble()),
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

  /**
   * Sets the storage's motor, which moves the belt, to the specified value
   *
   * @param power the specified power of the storage's motor as a percentage of full power [-1.0,
   *     1.0]; positive values rotate the belt in the intake direction
   */
  public void setStoragePower(double power) {
    io.setMotorPercentage(power);
  }

  /**
   * Enable the storage subsystem. This results in the storage's belt moving in the intake direction
   * at the default power.
   */
  public void enableStorage() {
    io.setEnabled(true);
    io.setMotorPercentage(StorageConstants.STORAGE_DEFAULT_POWER);
  }

  /**
   * Returns true if the storage subsystem is enabled (i.e., belt is moving in the intake direction
   * at the default power).
   *
   * @return true if the storage subsystem is enabled (i.e., belt is moving in the intake direction
   *     at the default power)
   */
  public boolean isStorageEnabled() {
    return inputs.isEnabled;
  }

  /** Disable the storage subsystem. This results in the storage's belt stopping. */
  public void disableStorage() {
    io.setEnabled(false);
    io.setMotorPercentage(0.0);
  }

  /**
   * Returns true if the sensor at the shooter end of the storage detects cargo.
   *
   * @return true if the sensor at the shooter end of the storage detects cargo
   */
  public boolean isShooterSensorUnblocked() {
    return inputs.isShooterSensorUnblocked;
  }

  /**
   * Returns true if the sensor at the collector end of the storage detects cargo.
   *
   * @return true if the sensor at the collector end of the storage detects cargo
   */
  public boolean isCollectorSensorUnblocked() {
    return inputs.isCollectorSensorUnblocked;
  }
}
