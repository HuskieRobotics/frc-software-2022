package frc.robot.subsystems.secondary_arm;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.secondary_arm.SecondaryArmIO.SecondaryArmIOInputs;

/**
 * This subsystem models the robot's secondary arms which hold the robot on a rung. It consists of a
 * solenoid which, when enabled, deploys the arms to engage with a rung; and, when disabled,
 * retracts the arms to remain clear of the rungs.
 */
public class SecondaryArm extends SubsystemBase {
  private final SecondaryArmIO io;
  private final SecondaryArmIOInputs inputs = new SecondaryArmIOInputs();

  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  /** Constructs a new SecondaryArm object. */
  public SecondaryArm(SecondaryArmIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    if (DEBUGGING) {
      tab.add("Secondary Arm", this);
      tab.addBoolean("Secondary Arms In?", this::isIn);
    }

    if (TESTING) {
      tab.add("Second Arm Out", new InstantCommand(() -> this.moveSecondaryArmOut(), this));
      tab.add("Second Arm In", new InstantCommand(() -> this.moveSecondaryArmIn(), this));
    }
  }

  /**
   * Returns true if the secondary arms are retracted (i.e., within the robot's frame)
   *
   * @return true if the secondary arms are retracted (i.e., within the robot's frame)
   */
  public boolean isIn() {
    return inputs.isIn;
  }

  /** Move the secondary arms within the robot's frame, which keeps them clear of rungs. */
  public void moveSecondaryArmIn() {
    io.setExtended(false);
  }

  /** Move the secondary arms out such that they can engage with a rung and support the robot. */
  public void moveSecondaryArmOut() {
    io.setExtended(true);
  }
}
