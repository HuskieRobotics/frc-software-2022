package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.StorageConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SortStorageCommand;
import java.util.Map;

/**
 * This subsystem models the robot's storage mechanism. Is consists of a single motor, which moves
 * the storage's belt in an intake or outtake direction, and two sensors which detect cargo at the
 * collector end and the shooter end of the storage.
 */
public class Storage extends SubsystemBase {
  private WPI_TalonSRX beltMotor;
  private DigitalInput collectorSensor;
  private DigitalInput shooterSensor;
  private boolean isStorageEnabled;

  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new Storage object. */
  public Storage() {
    this.isStorageEnabled = false;

    this.beltMotor = new WPI_TalonSRX(StorageConstants.STORAGE_MOTOR_ID);

    // no data is read from the Talon SRX; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.beltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.beltMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    this.collectorSensor = new DigitalInput(StorageConstants.COLLECTOR_SENSOR);

    shooterSensor = new DigitalInput(StorageConstants.SHOOTER_SENSOR);

    addChild("Storage Belt Motor", beltMotor);
    addChild("Storage Collector Sensor", collectorSensor);
    addChild("Storage Shooter Sensor", shooterSensor);

    ShuffleboardTab tab = Shuffleboard.getTab("Storage");

    if (DEBUGGING) {
      tab.add("Storage", this);
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
   * Sets the storage's motor, which moves the belt, to the specified value
   *
   * @param power the specified power of the storage's motor as a percentage of full power [-1.0,
   *     1.0]; positive values rotate the belt in the intake direction
   */
  public void setStoragePower(double power) {
    this.beltMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Enable the storage subsystem. This results in the storage's belt moving in the intake direction
   * at the default power.
   */
  public void enableStorage() {
    this.isStorageEnabled = true;
    this.beltMotor.set(ControlMode.PercentOutput, StorageConstants.STORAGE_DEFAULT_SPEED);
  }

  /**
   * Returns true if the storage subsystem is enabled (i.e., belt is moving in the intake direction
   * at the default power).
   *
   * @return true if the storage subsystem is enabled (i.e., belt is moving in the intake direction
   *     at the default power)
   */
  public boolean isStorageEnabled() {
    return isStorageEnabled;
  }

  /** Disable the storage subsystem. This results in the storage's belt stopping. */
  public void disableStorage() {
    this.isStorageEnabled = false;
    this.beltMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Returns true if the sensor at the shooter end of the storage detects cargo.
   *
   * @return true if the sensor at the shooter end of the storage detects cargo
   */
  public boolean isShooterSensorUnblocked() {
    return this.shooterSensor.get();
  }

  /**
   * Returns true if the sensor at the collector end of the storage detects cargo.
   *
   * @return true if the sensor at the collector end of the storage detects cargo
   */
  public boolean isCollectorSensorUnblocked() {
    return this.collectorSensor.get();
  }
}
