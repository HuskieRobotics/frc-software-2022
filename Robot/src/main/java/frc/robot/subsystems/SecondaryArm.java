package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SecondMechanismConstants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Constants.*;

/**
 * This subsystem models the robot's secondary arms which hold the robot on a rung. It consists of
 * a solenoid which, when enabled, deploys the arms to engage with a rung; and, when disabled,
 * retracts the arms to remain clear of the rungs.
 */
public class SecondaryArm extends SubsystemBase {
    private boolean isIn;
    private Solenoid secondaryMechanism;

    public SecondaryArm() {
        this.isIn = false;

        secondaryMechanism = new Solenoid(SecondMechanismConstants.PNEUMATIC_HUB_CAN_ID, PneumaticsModuleType.REVPH,
                SecondMechanismConstants.PNEUMATIC_CHANNEL);
        addChild("Secondary Mechanism", this.secondaryMechanism);

        if(COMMAND_LOGGING) {
            Shuffleboard.getTab("Elevator").add("Second Arm Out", new InstantCommand(this::moveSecondaryArmOut));
            Shuffleboard.getTab("Elevator").add("Second Arm In", new InstantCommand(this::moveSecondaryArmIn));
            Shuffleboard.getTab("Elevator").addBoolean("Second Arm In?", this::isIn);
        }
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public boolean isIn() {
        return this.isIn;
    }

    /**
     * Move the secondary arms within the robot's frame, which keeps them clear of rungs.
     */
    public void moveSecondaryArmIn() {
        this.secondaryMechanism.set(false);
        this.isIn = true;
    }

    /**
     * Move the secondary arms out such that they can engage with a rung and support the robot.
     */
    public void moveSecondaryArmOut() {
        this.secondaryMechanism.set(true);
        this.isIn = false;
    }

}
