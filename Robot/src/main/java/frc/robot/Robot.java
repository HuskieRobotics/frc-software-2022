package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.StorageConstants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
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

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit() {

    // from AdvantageKit Robot Configuration docs
    // (https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md#robot-configuration)

    // Run as fast as possible during replay
    setUseTiming(isReal());

    // Log & replay "SmartDashboard" values (no tables are logged by default).
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");

    // Set a metadata value
    Logger.getInstance().recordMetadata("ProjectName", "frc-software-2022");

    if (isReal()) {
      // Log to USB stick (name will be selected automatically)
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/"));

      // Provide log data over the network, viewable in Advantage Scope.
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    } else {
      // Prompt the user for a file path on the command line
      String path = ByteLogReplay.promptForPath();

      // Read log file for replay
      Logger.getInstance().setReplaySource(new ByteLogReplay(path));

      // Save replay results to a new log with the "_sim" suffix
      Logger.getInstance()
          .addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    Logger.getInstance().start();

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
