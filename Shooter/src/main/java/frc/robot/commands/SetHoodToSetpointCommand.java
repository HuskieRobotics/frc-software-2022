package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class SetHoodToSetpointCommand extends PIDCommand{
   
    
    public SetHoodToSetpointCommand(Hood hood) {
        /*super(
            new PIDController(HoodConstants.HOOD_P, 0, 0), 
            hood :: getHoodEncoder, 
            0,
            output -> Hood.setHoodMotorPower(output), 
            hood); */
            super(
                new PIDController(HoodConstants.HOOD_P, 0, 0),
                hood :: getHoodEncoderValue,
                hood.getHoodSetpointLimeLight(),
                output -> hood.setHoodMotorPower(output), 
                hood);
        }
    
  
}