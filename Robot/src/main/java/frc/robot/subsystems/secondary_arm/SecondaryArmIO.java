// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.secondary_arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface SecondaryArmIO {
  /** Contains all of the input data received from hardware. */
  public static class SecondaryArmIOInputs implements LoggableInputs {
    boolean isIn = false;

    public void toLog(LogTable table) {
      table.put("isIn", isIn);
    }

    public void fromLog(LogTable table) {
      isIn = table.getBoolean("Extended", isIn);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SecondaryArmIOInputs inputs) {}

  /** Set solenoid state. */
  public default void setExtended(boolean extended) {}
}
