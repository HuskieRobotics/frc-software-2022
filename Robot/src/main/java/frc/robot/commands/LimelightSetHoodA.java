package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightMath;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.Constants.HoodConstants;

public class LimelightSetHoodA extends CommandBase {
    private double position;
    private LimelightMath limelightMath;
    private Hood hood;

    public LimelightSetHoodA(Hood h, LimelightMath lm) {
        hood = h;
        limelightMath = lm;
    }

    @Override
    public void initialize() {
        double angle = limelightMath.getIdealHoodA();
        position= angle*HoodConstants.HOOD_DEGREES_TO_HOOD_ENCODER;
    }

    @Override
    public void execute() {
        hood.setHoodSetpoint(position);
    }
}