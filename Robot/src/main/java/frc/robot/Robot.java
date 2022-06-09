package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private UsbCamera storageCam;
  private VideoSink server;
  private ByteLogReceiver logReceiver;

  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert logOpenFileAlert =
      new Alert("Failed to open log file. Data will NOT be logged", AlertType.ERROR);
  private final Alert logWriteAlert =
      new Alert("Failed write to the log file. Data will NOT be logged", AlertType.ERROR);

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {
    final String GIT_DIRTY = "GitDirty";

    // from AdvantageKit Robot Configuration docs
    // (https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md#robot-configuration)

    Logger logger = Logger.getInstance();

    // Run as fast as possible during replay
    setUseTiming(isReal());

    // Log & replay "SmartDashboard" values (no tables are logged by default).
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");

    // Set a metadata value
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", GVersion.MAVEN_NAME);
    logger.recordMetadata("BuildDate", GVersion.BUILD_DATE);
    logger.recordMetadata("GitSHA", GVersion.GIT_SHA);
    logger.recordMetadata("GitDate", GVersion.GIT_DATE);
    logger.recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    switch (GVersion.DIRTY) {
      case 0:
        logger.recordMetadata(GIT_DIRTY, "All changes committed");
        break;
      case 1:
        logger.recordMetadata(GIT_DIRTY, "Uncomitted changes");
        break;
      default:
        logger.recordMetadata(GIT_DIRTY, "Unknown");
        break;
    }

    if (isReal()) {
      // Log to USB stick (name will be selected automatically)
      logReceiver = new ByteLogReceiver("/media/sda1/");
      logger.addDataReceiver(logReceiver);

      // Provide log data over the network, viewable in Advantage Scope.
      logger.addDataReceiver(new LogSocketServer(5800));

      LoggedSystemStats.getInstance()
          .setPowerDistributionConfig(
              DrivetrainConstants.POWER_DISTRIBUTION_HUB_ID, ModuleType.kRev);
    } else {
      // Prompt the user for a file path on the command line
      String path = ByteLogReplay.promptForPath();

      // Read log file for replay
      logger.setReplaySource(new ByteLogReplay(path));

      // Save replay results to a new log with the "_sim" suffix
      logger.addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    logger.start();

    // Alternative logging of scheduled commands
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> Logger.getInstance().recordOutput("Command initialized", command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> Logger.getInstance().recordOutput("Command interrupted", command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> Logger.getInstance().recordOutput("Command finished", command.getName()));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = RobotContainer.getInstance();

    storageCam = CameraServer.startAutomaticCapture(StorageConstants.STORAGE_CAMERA_PORT);
    storageCam.setResolution(320, 240);
    storageCam.setFPS(15);
    storageCam.setPixelFormat(PixelFormat.kYUYV);
    storageCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    server = CameraServer.getServer();

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing finished or
    // interrupted commands, and running subsystem periodic() methods. This must be called from the
    // robot's periodic block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Log scheduled commands
    Logger.getInstance()
        .recordOutput(
            "ActiveCommands/Scheduler",
            NetworkTableInstance.getDefault()
                .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
                .getStringArray(new String[] {}));

    // Check logging faults
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());
    if (logReceiver != null) {
      logOpenFileAlert.set(logReceiver.getOpenFault());
      logWriteAlert.set(logReceiver.getWriteFault());
    }
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.getInstance().disabledPeriodic();
  }

  /**
   * This autonomous schedules the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This method is called once each time the robot enters Teleop mode. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    server.setSource(storageCam);
    robotContainer.teleopInit();
  }

  /** This method is called once each time the robot enters Test mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
