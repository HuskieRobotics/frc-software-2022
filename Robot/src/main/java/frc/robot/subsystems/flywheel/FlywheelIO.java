// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface FlywheelIO {
  /** Contains all of the input data received from hardware. */
  public static class FlywheelIOInputs implements LoggableInputs {
    double leftVelocity = 0.0;
    double leftClosedLoopError = 0.0;
    double leftAppliedVolts = 0.0;
    double[] leftCurrentAmps = new double[] {};
    double[] leftTempCelcius = new double[] {};

    double rightVelocity = 0.0;
    double rightClosedLoopError = 0.0;
    double rightAppliedVolts = 0.0;
    double[] rightCurrentAmps = new double[] {};
    double[] rightTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("LeftVelocity", leftVelocity);
      table.put("LeftClosedLoopError", leftClosedLoopError);
      table.put("LeftAppliedVolts", leftAppliedVolts);
      table.put("LeftCurrentAmps", leftCurrentAmps);
      table.put("LeftTempCelcius", leftTempCelcius);

      table.put("RightVelocity", rightVelocity);
      table.put("RightClosedLoopError", rightClosedLoopError);
      table.put("RightAppliedVolts", rightAppliedVolts);
      table.put("RightCurrentAmps", rightCurrentAmps);
      table.put("RightTempCelcius", rightTempCelcius);
    }

    public void fromLog(LogTable table) {
      leftVelocity = table.getDouble("LeftVelocity", leftVelocity);
      leftClosedLoopError = table.getDouble("LeftClosedLoopError", leftClosedLoopError);
      leftAppliedVolts = table.getDouble("LeftAppliedVolts", leftAppliedVolts);
      leftCurrentAmps = table.getDoubleArray("LeftCurrentAmps", leftCurrentAmps);
      leftTempCelcius = table.getDoubleArray("LeftTempCelcius", leftTempCelcius);

      rightVelocity = table.getDouble("RightVelocity", rightVelocity);
      rightClosedLoopError = table.getDouble("RightClosedLoopError", rightClosedLoopError);
      rightAppliedVolts = table.getDouble("RightAppliedVolts", rightAppliedVolts);
      rightCurrentAmps = table.getDoubleArray("RightCurrentAmps", rightCurrentAmps);
      rightTempCelcius = table.getDoubleArray("RightTempCelcius", rightTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run the flywheel open loop at the specified voltage. */
  public default void setMotorPercentage(double percentage) {}

  /** Run the flywheel closed loop at the specified velocity. */
  public default void setVelocity(double velocity) {}

  /** Set velocity feed forward constant. */
  public default void configureKF(double kF) {}

  /** Set velocity proportional constant. */
  public default void configureKP(double kP) {}

  /** Set velocity integrative constant. */
  public default void configureKI(double kI) {}

  /** Set velocity derivative constant. */
  public default void configureKD(double kD) {}
}
