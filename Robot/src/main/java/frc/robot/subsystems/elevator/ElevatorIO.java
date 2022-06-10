// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class ElevatorIOInputs implements LoggableInputs {
    boolean isControlEnabled = false;

    double leftPosition = 0.0;
    double leftVelocity = 0.0;
    double leftClosedLoopError = 0.0;
    double leftAppliedVolts = 0.0;
    double[] leftCurrentAmps = new double[] {};
    double[] leftTempCelcius = new double[] {};

    double rightPosition = 0.0;
    double rightVelocity = 0.0;
    double rightClosedLoopError = 0.0;
    double rightAppliedVolts = 0.0;
    double[] rightCurrentAmps = new double[] {};
    double[] rightTempCelcius = new double[] {};

    double pitch = 0.0;

    public void toLog(LogTable table) {
      table.put("ControlEnabled", isControlEnabled);

      table.put("LeftPosition", leftPosition);
      table.put("LeftVelocity", leftVelocity);
      table.put("LeftClosedLoopError", leftClosedLoopError);
      table.put("LeftAppliedVolts", leftAppliedVolts);
      table.put("LeftCurrentAmps", leftCurrentAmps);
      table.put("LeftTempCelcius", leftTempCelcius);

      table.put("RightPosition", rightPosition);
      table.put("RightVelocity", rightVelocity);
      table.put("RightClosedLoopError", rightClosedLoopError);
      table.put("RightAppliedVolts", rightAppliedVolts);
      table.put("RightCurrentAmps", rightCurrentAmps);
      table.put("RightTempCelcius", rightTempCelcius);

      table.put("Pitch", pitch);
    }

    public void fromLog(LogTable table) {
      isControlEnabled = table.getBoolean("ControlEnabled", isControlEnabled);

      leftPosition = table.getDouble("LeftPosition", leftPosition);
      leftVelocity = table.getDouble("LeftVelocity", leftVelocity);
      leftClosedLoopError = table.getDouble("LeftClosedLoopError", leftClosedLoopError);
      leftAppliedVolts = table.getDouble("LeftAppliedVolts", leftAppliedVolts);
      leftCurrentAmps = table.getDoubleArray("LeftCurrentAmps", leftCurrentAmps);
      leftTempCelcius = table.getDoubleArray("LeftTempCelcius", leftTempCelcius);

      rightPosition = table.getDouble("RightPosition", rightPosition);
      rightVelocity = table.getDouble("RightVelocity", rightVelocity);
      rightClosedLoopError = table.getDouble("RightClosedLoopError", rightClosedLoopError);
      rightAppliedVolts = table.getDouble("RightAppliedVolts", rightAppliedVolts);
      rightCurrentAmps = table.getDoubleArray("RightCurrentAmps", rightCurrentAmps);
      rightTempCelcius = table.getDoubleArray("RightTempCelcius", rightTempCelcius);

      pitch = table.getDouble("Pitch", pitch);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Enable/Disable collector. */
  public default void setControlEnabled(boolean controlEnabled) {}

  /** Run the climber open loop at the specified voltage. */
  public default void setMotorPercentage(double percentage) {}

  /** Run the climber closed loop to the specified position. */
  public default void setPosition(double position, double arbitraryFeedForward) {}

  /** Set position feed forward constant. */
  public default void configureKF(double kF) {}

  /** Set position proportional constant. */
  public default void configureKP(double kP) {}

  /** Set position integrative constant. */
  public default void configureKI(double kI) {}

  /** Set position derivative constant. */
  public default void configureKD(double kD) {}

  /** Set closed loop peak output. */
  public default void configClosedLoopPeakOutput(double peakOutput) {}
}
