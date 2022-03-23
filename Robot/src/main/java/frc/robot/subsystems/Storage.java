package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.StorageConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 */
public class Storage extends SubsystemBase {
    private WPI_TalonSRX storage4;
    private DigitalInput collectorSensor0;
    private DigitalInput shooterSensor1;
    private boolean isStorageEnabled;

    
    /**
    *
    */
    public Storage() {
        this.isStorageEnabled = false;
        storage4 = new WPI_TalonSRX(StorageConstants.STORAGE_MOTOR_ID);

        this.storage4.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, TIMEOUT_MS);
        this.storage4.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, TIMEOUT_MS);

        collectorSensor0 = new DigitalInput(StorageConstants.COLLECTOR_SENSOR);
        addChild("Collector Sensor 0", collectorSensor0);

        shooterSensor1 = new DigitalInput(StorageConstants.SHOOTER_SENSOR);
        addChild("Shooter Sensor 1", shooterSensor1);

        
        if(COMMAND_LOGGING) {
            Shuffleboard.getTab("MAIN").addBoolean("Collector Unblocked", this::isCollectorSensorUnblocked);
            Shuffleboard.getTab("MAIN").addBoolean("Shooter Unblocked", this::isShooterSensorUnblocked);
            Shuffleboard.getTab("Storage").add("storage", this);
        }

        if (TUNING) {
            // Each robot feature that requires PID tuniing has its own Shuffleboard tab for
            // tuning (i.e., "ShooterTuning")
            // Add indicators and controls to this Shuffleboard tab to assist with
            // interactively tuning the system.

            Shuffleboard.getTab("Storage")
                    .add("Storage Power", 0.0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .withProperties(Map.of("min", -1.0, "max", 1.0)) // specify widget properties here
                    .getEntry()
                    .addListener(event -> {
                        this.setStoragePower(event.getEntry().getValue().getDouble());
                    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
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
    public void setStoragePower(double power) {
        this.storage4.set(ControlMode.PercentOutput, power);
    }

    public void enableStorage() {
        this.isStorageEnabled = true;
        this.storage4.set(ControlMode.PercentOutput, StorageConstants.STORAGE_DEFAULT_SPEED);
        

    }
    public boolean isStorageEnabled() { 
        return isStorageEnabled;
    }

    public void disableStorage() {
        this.isStorageEnabled = false;
        this.storage4.set(ControlMode.PercentOutput, 0);
    }

    public boolean isShooterSensorUnblocked() {
        return this.shooterSensor1.get();
    }

    public boolean isCollectorSensorUnblocked() {
        return this.collectorSensor0.get();
    }

    public double getStoragePower() {
        return this.storage4.get();
    }
}
