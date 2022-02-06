package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter;

public class SetShooterVelocity extends CommandBase{
    private shooter st;
    public SetShooterVelocity(shooter shooter) {
        st = shooter;
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double velo = st.getShooterVeloMap(distance);
        st.setFlywheelVelo(velo);
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

