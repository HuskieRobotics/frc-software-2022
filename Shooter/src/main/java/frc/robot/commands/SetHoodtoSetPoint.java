package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ShooterConstants;;

public class SetHoodtoSetPoint extends PIDCommand{
    public SetHoodtoSetPoint(shooter shooter, int setpoint) {
        super(
            new PIDController(ShooterConstants.HOOD_P, 0, 0), 
            shooter :: getHoodEncoder, 
            setpoint, 
            output -> shooter.setHoodPower(output), 
            shooter); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}