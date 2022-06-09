// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface CollectorIO {
  /** Contains all of the input data received from hardware. */
  public static class CollectorIOInputs implements LoggableInputs {
    boolean isEnabled = false;
    double appliedVolts = 0.0;
    double[] currentAmps = new double[] {};
    double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Enabled", isEnabled);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      isEnabled = table.getBoolean("Extended", isEnabled);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CollectorIOInputs inputs) {}

  /** Enable/Disable collector. */
  public default void setEnabled(boolean enabled) {}

  /** Run the roller open loop at the specified voltage. */
  public default void setMotorPercentage(double percentage) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
