// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// from https://github.com/Mechanical-Advantage/RobotCode2022

package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Pneumatics subsystem hardware interface. */
public interface PneumaticsIO {
  /** Contains all of the input data received from hardware. */
  public static class PneumaticsIOInputs implements LoggableInputs {
    double revPressurePsi = 0.0;
    boolean compressorActive = false;
    double compressorCurrentAmps = 0.0;
    double pressurePsi = 0.0;
    double flow = 0.0;

    public void toLog(LogTable table) {
      table.put("RevPressurePsi", revPressurePsi);
      table.put("CompressorActive", compressorActive);
      table.put("CompressorCurrentAmps", compressorCurrentAmps);
      table.put("PressurePsi", pressurePsi);
      table.put("Flow", flow);
    }

    public void fromLog(LogTable table) {
      revPressurePsi = table.getDouble("RevPressurePsi", revPressurePsi);
      compressorActive = table.getBoolean("CompressorActive", compressorActive);
      compressorCurrentAmps = table.getDouble("CompressorCurrentAmps", compressorCurrentAmps);
      pressurePsi = table.getDouble("PressurePsi", pressurePsi);
      flow = table.getDouble("Flow", flow);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PneumaticsIOInputs inputs) {}

  /** Updates the compressor threshold */
  public default void useLowClosedLoopThresholds(boolean useLow) {}
}
