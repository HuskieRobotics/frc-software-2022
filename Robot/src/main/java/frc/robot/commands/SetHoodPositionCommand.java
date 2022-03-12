package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightMath;
import frc.robot.subsystems.Hood;
import static frc.robot.Constants.FlywheelConstants.*;

public class SetHoodPositionCommand extends CommandBase{
    private Hood hood;
    private double position;
    public SetHoodPositionCommand(Hood hood, double position) {
        this.hood = hood;
        // FIXME: this will get the ideal velocity when the command is constructed; not when the command is scheduled
        //      The next line should be moved to the initialzie method.
        this.position = position;
        addRequirements(this.hood);

        
    }
    

    // Called when the command is initially scheduled.

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hood.setHoodSetpoint(this.position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hood.setHoodMotorPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hood.isAtSetpoint();
    }
}