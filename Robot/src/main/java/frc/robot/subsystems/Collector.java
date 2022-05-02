package frc.robot.subsystems;

import static frc.robot.Constants.CollectorConstants.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class Collector extends SubsystemBase {
    private WPI_TalonSRX collector5;
    private Solenoid collectorPiston;
    private boolean isEnabled;
    private NetworkTableEntry collectorMotorPowerNT;

    /**
    *
    */
    public Collector() {
        isEnabled = false; //collector state

        collector5 = new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_ID);
        collector5.setInverted(true);


        //setting the status frame period controlls CAN bus utilization
        this.collector5.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
        this.collector5.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

        collectorPiston = new Solenoid(CollectorConstants.PEUNAMATICS_HUB_CAN_ID, PneumaticsModuleType.REVPH,
                CollectorConstants.COLLECTOR_SOLENOID_CHANNEL);
        addChild("Collector Piston", collectorPiston);

        this.collectorMotorPowerNT = Shuffleboard.getTab("Collector")
                .add("Collector speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .getEntry();


        //shuffleboard information useful for debugging but not during regular use
        if(COMMAND_LOGGING) {
            Shuffleboard.getTab("Collector").add("collector", this);
            Shuffleboard.getTab("Collector").add("deployCollector", new InstantCommand(this::deployCollectorPiston, this));
            Shuffleboard.getTab("Collector").add("retractCollector",
                    new InstantCommand(this::retractCollectorPiston, this));
            Shuffleboard.getTab("Collector").addBoolean("enabled", this::isEnabled);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //for tuning pid's or any other "tuning"
        if (TUNING) {
            double collectorPower = this.collectorMotorPowerNT.getDouble(0.0);
            this.setCollectorPower(collectorPower);
        }

    }


    // collector motor controll
    public void setCollectorPower(double power) {
        this.collector5.set(ControlMode.PercentOutput, power);
    }



    public void disableCollector() {
        isEnabled = false;
        this.collector5.set(ControlMode.PercentOutput, 0);
        this.collectorPiston.set(false);
    }

    public void enableCollector() {
        isEnabled = true;
        this.collector5.set(ControlMode.PercentOutput, CollectorConstants.COLLECTOR_DEFUALT_SPEED);
        this.collectorPiston.set(true);
    }


    //accessor for the collector state
    public boolean isEnabled() {
        return isEnabled;
    }


    //pistion control
    public void deployCollectorPiston() {
        this.collectorPiston.set(true);
    }

    public void retractCollectorPiston() {
        this.collectorPiston.set(false);
    }


    //pistion state
    public boolean isPistonExtended() {
        return this.collectorPiston.get();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
