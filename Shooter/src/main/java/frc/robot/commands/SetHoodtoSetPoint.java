package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;

public class SetHoodtoSetPoint extends PIDCommand{
   
    
    public SetHoodtoSetPoint(shooter shooter) {
        super(
            new PIDController(ShooterConstants.HOOD_P, 0, 0), 
            shooter :: getHoodEncoder, 
            shooter.getHoodSetpoint(), 
            output -> shooter.setHoodMotorPower(output), 
            shooter); 
        }
    
  
}