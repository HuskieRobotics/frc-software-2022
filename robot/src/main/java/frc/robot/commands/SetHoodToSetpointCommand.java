package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class SetHoodToSetpointCommand extends CommandBase{
    private Hood hood;
    public SetHoodToSetpointCommand(Hood hood) {
        
           this.hood = hood;


        }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        hood.setHoodSetPoint(hood.getHoodSetpointLimeLight());  // Method is in a issue, should work once that issue is resolved

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}