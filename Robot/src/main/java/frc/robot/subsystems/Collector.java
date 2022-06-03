package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem models the robot's collector mechanism. It consists of a single motor, which
 * rotates the collector's intake wheels in an intake or outtake direction, and a solenoid which,
 * when enabled, deploys the collector; and, when disabled, retracts the collector.
 */
public class Collector extends SubsystemBase {
  private WPI_TalonSRX collector5;
  private Solenoid collectorPiston;
  private boolean isEnabled;
  private NetworkTableEntry collectorMotorPowerNT;

  /** Constructs a new Collector object. */
  public Collector() {
    isEnabled = false;
    collector5 = new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_ID);
    collector5.setInverted(true);

    // no data is read from the Talon SRX; so, set these CAN frame periods to the maximum value
    //  to reduce traffic on the bus
    this.collector5.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
    this.collector5.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

    collectorPiston =
        new Solenoid(
            CollectorConstants.PEUNAMATICS_HUB_CAN_ID,
            PneumaticsModuleType.REVPH,
            CollectorConstants.COLLECTOR_SOLENOID_CHANNEL);
    // should all sensors and actuators in a subsystem be added as children of that subsystem? what
    // benefit does that provide?
    addChild("Collector Piston", collectorPiston);

    // update to the approach in the Flywheel class
    this.collectorMotorPowerNT =
        Shuffleboard.getTab("Collector")
            .add("Collector speed", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();

    if (COMMAND_LOGGING) {
      Shuffleboard.getTab("Collector").add("collector", this);
      // replace with lambda expression and eliminate methods
      Shuffleboard.getTab("Collector")
          .add("deployCollector", new InstantCommand(this::deployCollectorPiston, this));
      Shuffleboard.getTab("Collector")
          .add("retractCollector", new InstantCommand(this::retractCollectorPiston, this));
      Shuffleboard.getTab("Collector").addBoolean("enabled", this::isEnabled);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (TUNING) {
      // update to the approach in the Flywheel class
      double collectorPower = this.collectorMotorPowerNT.getDouble(0.0);
      this.setCollectorPower(collectorPower);
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
    this.collector5.set(ControlMode.PercentOutput, power);
  }

  /**
   * Disables the collector subsystem. This results in stopping the collector's intake wheels and
   * retracting the collector back inside the robot frame.
   */
  public void disableCollector() {
    isEnabled = false;
    this.collector5.set(ControlMode.PercentOutput, 0);
    this.collectorPiston.set(false);
  }

  /**
   * Enables the collector subsystem. This results in spinning the collector's intake wheels in the
   * intake direcion and extending the collector outside the robot frame to position it to collect
   * cargo.
   */
  public void enableCollector() {
    isEnabled = true;
    this.collector5.set(ControlMode.PercentOutput, CollectorConstants.COLLECTOR_DEFUALT_SPEED);
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

  public void deployCollectorPiston() {
    this.collectorPiston.set(true);
  }

  public void retractCollectorPiston() {
    this.collectorPiston.set(false);
  }

  // delete
  public boolean isPistonExtended() {
    return this.collectorPiston.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
