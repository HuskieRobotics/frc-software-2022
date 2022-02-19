package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveForward extends PIDCommand {

    public AutoDriveForward(double distance, DrivetrainSubsystem drive) {
        super(
            new PIDController(1,0,0),
            drive::getDistance,
            distance,
            output -> drive.setMotorPower(output),
            drive);

        getController().setTolerance(.1, .1);
        
    }

    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }
    
}
