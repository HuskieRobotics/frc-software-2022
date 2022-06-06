package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem models the robot's collector mechanism. It consists of a single motor, which
 * rotates the collector's intake wheels in an intake or outtake direction, and a solenoid which,
 * when enabled, deploys the collector; and, when disabled, retracts the collector.
 */
public class Collector extends SubsystemBase {
  private WPI_TalonSRX collectorMotor;
  private Solenoid collectorPiston;
  private boolean isEnabled;

  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new Collector object. */
  public Collector() {
    isEnabled = false;

    collectorMotor = new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_ID);
    collectorMotor.setInverted(true);

    // no data is read from the Talon SRX; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    collectorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    collectorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    collectorPiston =
        new Solenoid(
            CollectorConstants.PEUNAMATICS_HUB_CAN_ID,
            PneumaticsModuleType.REVPH,
            CollectorConstants.COLLECTOR_SOLENOID_CHANNEL);

    addChild("Collector Piston", collectorPiston);
    addChild("Collector Motor", collectorMotor);

    ShuffleboardTab tab = Shuffleboard.getTab("Collector");

    if (DEBUGGING) {
      tab.add("Collector", this);
      tab.addBoolean("Enabled?", this::isEnabled);
    }

    if (TESTING) {
      tab.add("Collector Speed", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry()
          .addListener(
              event ->
                  collectorMotor.set(
                      ControlMode.PercentOutput, event.getEntry().getValue().getDouble()),
              EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      tab.add("Deploy Collector", new InstantCommand(() -> this.collectorPiston.set(true), this));
      tab.add("Retract Collector", new InstantCommand(() -> this.collectorPiston.set(false), this));
    }
  }

  /**
   * Sets the power of the collector's motor, which rotates the collector's intake wheels, to the
   * specified value.
   *
   * @param power the specified power of the collector's motor as a percentage of full power [-1.0,
   *     1.0]; positive values rotate the wheels in the intake direction
   */
  public void setCollectorPower(double power) {
    this.collectorMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Disables the collector subsystem. This results in stopping the collector's intake wheels and
   * retracting the collector back inside the robot frame.
   */
  public void disableCollector() {
    isEnabled = false;
    this.collectorMotor.set(ControlMode.PercentOutput, 0);
    this.collectorPiston.set(false);
  }

  /**
   * Enables the collector subsystem. This results in spinning the collector's intake wheels in the
   * intake direcion and extending the collector outside the robot frame to position it to collect
   * cargo.
   */
  public void enableCollector() {
    isEnabled = true;
    this.collectorMotor.set(ControlMode.PercentOutput, CollectorConstants.COLLECTOR_DEFUALT_SPEED);
    this.collectorPiston.set(true);
  }

  /**
   * Returns true if the collector is enabled (i.e., intake wheels spinning and collector deployed).
   *
   * @return true if the collector is enabled (i.e., intake wheels spinning and collector deployed)
   */
  public boolean isEnabled() {
    return isEnabled;
  }
}
