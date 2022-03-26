// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that does nothing but takes a specified amount of time to finish. Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal timer.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitForTimedShotCommand extends CommandBase {
  protected Timer m_timer = new Timer();
  private final double m_duration;
  private DrivetrainSubsystem drivetrain;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  public WaitForTimedShotCommand(DrivetrainSubsystem drivetrainSubsystem, double seconds) {
    m_duration = seconds;
    drivetrain = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  public void execute() {
      drivetrain.drive(0, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
