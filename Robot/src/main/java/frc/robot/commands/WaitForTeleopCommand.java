package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Storage;

public class WaitForTeleopCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Collector collector;
    private final Storage storage;
    private final Flywheel flywheel;

    public WaitForTeleopCommand(DrivetrainSubsystem drivetrainSubsystem, Flywheel flywheel, Storage storage, Collector collector) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.collector = collector;
        this.storage = storage;
        this.flywheel = flywheel;
        addRequirements(drivetrainSubsystem);
        addRequirements(collector);
        addRequirements(storage);
        addRequirements(flywheel);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.disableXstance();
        storage.disableStorage();
        flywheel.stopFlywheel();
        collector.disableCollector();
    }
}
