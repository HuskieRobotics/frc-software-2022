// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// from https://github.com/Mechanical-Advantage/RobotCode2022

package frc.robot.subsystems.pneumatics;

import static frc.robot.Constants.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticHub;

public class PneumaticsIORev implements PneumaticsIO {
  private static final double PRESSURE_SUPPLY_NORMALIZED = 4.5868055556; // FIX_ME
  private static final double FLOW_SUPPLY_NORMALIZED = 4.5868055556; // FIX_ME

  private final PneumaticHub pneumatics;
  private final AnalogInput pressureSensor;
  private final AnalogInput flowSensor;

  public PneumaticsIORev() {
    pneumatics = new PneumaticHub(PNEUMATICS_HUB_ID);
    pressureSensor = new AnalogInput(PRESSURE_SENSOR_CHANNEL);
    flowSensor = new AnalogInput(FLOW_SENSOR_CHANNEL);
    useLowClosedLoopThresholds(false);
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.revPressurePsi = pneumatics.getPressure(0);
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();
    inputs.pressurePsi =
        ((pressureSensor.getAverageVoltage() / PRESSURE_SUPPLY_NORMALIZED) * 250) - 25; // FIX_ME
    inputs.flow = ((flowSensor.getAverageVoltage() / FLOW_SUPPLY_NORMALIZED) * 250) - 25; // FIX_ME
  }

  @Override
  public void useLowClosedLoopThresholds(boolean useLow) {
    if (useLow) {
      pneumatics.enableCompressorAnalog(50, 60);
    } else {
      pneumatics.enableCompressorAnalog(80, 120);
    }
  }
}
