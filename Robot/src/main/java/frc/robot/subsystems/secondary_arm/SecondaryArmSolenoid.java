// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.secondary_arm;

import static frc.robot.Constants.SecondaryArmConstants.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SecondaryArmSolenoid implements SecondaryArmIO {
  private final Solenoid solenoid;

  public SecondaryArmSolenoid() {

    solenoid = new Solenoid(PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH, PNEUMATIC_CHANNEL);
  }

  @Override
  public void updateInputs(SecondaryArmIOInputs inputs) {
    inputs.isIn = !solenoid.get();
  }

  @Override
  public void setExtended(boolean extended) {
    solenoid.set(extended);
  }
}
