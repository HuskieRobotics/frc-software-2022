// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.storage;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface StorageIO {
  /** Contains all of the input data received from hardware. */
  public static class StorageIOInputs implements LoggableInputs {
    boolean isEnabled = false;

    double appliedVolts = 0.0;
    double[] currentAmps = new double[] {};
    double[] tempCelcius = new double[] {};

    boolean isShooterSensorUnblocked = true;
    boolean isCollectorSensorUnblocked = true;

    public void toLog(LogTable table) {
      table.put("Enabled", isEnabled);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("ShooterSensorUnblocked", isShooterSensorUnblocked);
      table.put("CollectorSensorUnblocked", isCollectorSensorUnblocked);
    }

    public void fromLog(LogTable table) {
      isEnabled = table.getBoolean("Extended", isEnabled);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      isShooterSensorUnblocked =
          table.getBoolean("ShooterSensorUnblocked", isShooterSensorUnblocked);
      isCollectorSensorUnblocked =
          table.getBoolean("CollectorSensorUnblocked", isCollectorSensorUnblocked);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(StorageIOInputs inputs) {}

  /** Enable/Disable collector. */
  public default void setEnabled(boolean enabled) {}

  /** Run the roller open loop at the specified voltage. */
  public default void setMotorPercentage(double percentage) {}
}
