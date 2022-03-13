package frc.robot;

import static frc.robot.Constants.JoystickConstants.BUTTON_B;
import static frc.robot.Constants.JoystickConstants.BUTTON_X;
import static frc.robot.Constants.JoystickConstants.BUTTON_Y;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendClimberToMidRungCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ReachToNextRungCommand;
import frc.robot.commands.RetractClimberFullCommand;
import frc.robot.commands.RetractClimberMinimumCommand;
import frc.robot.commands.SortStorageCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightMath;
import frc.robot.subsystems.SecondaryArm;
import frc.robot.subsystems.Storage;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  private final JoystickButton[] operatorButtons;
  private final JoystickButton[] joystickButtons0;
  private final JoystickButton[] joystickButtons1;
  private final JoystickButton[] xboxButtons;

  private final Joystick joystick0 = new Joystick(0);
  private final Joystick joystick1 = new Joystick(1);
  private final Joystick operatorConsole = new Joystick(2);
  private final XboxController xboxController = new XboxController(3);

  private static RobotContainer m_robotContainer = new RobotContainer();

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Storage m_storage = new Storage();
  private final Collector m_collector = new Collector();
  private final Flywheel m_flywheel = new Flywheel();
  private final Hood m_hood = new Hood();
  private final LimelightMath m_limelight = new LimelightMath();
  private final SecondaryArm m_secondMechanism = null; // = new SecondaryArm();
  private final Elevator m_elevator = new Elevator();

  private Command autoLeaveTarmac;
  private Command autoBlue1;
  private Command autoRed1;

  // Joysticks

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];
    this.operatorButtons = new JoystickButton[13];
    this.xboxButtons = new JoystickButton[10];
    for (int i = 1; i < joystickButtons0.length; i++) {
      joystickButtons0[i] = new JoystickButton(joystick0, i);
      joystickButtons1[i] = new JoystickButton(joystick1, i);
      operatorButtons[i] = new JoystickButton(operatorConsole, i);
    }

    for (int i = 1; i < xboxButtons.length; i++) {
      xboxButtons[i - 1] = new JoystickButton(xboxController, i);
    }

    m_drivetrainSubsystem.register();
    m_storage.register();
    m_collector.register();
    m_flywheel.register();
    m_hood.register();
    m_limelight.register();
    m_storage.register();
    m_elevator.register();
    // m_secondMechanism.register();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // FIXME: remove the -1 after calibrating swerve with the new robot "front" and
        // chaning CAN IDs to match new orientation
        () -> -modifyAxis(joystick1.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * -1));

    // Smartdashboard Subsystems

    // SmartDashboard Buttons

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // Configure autonomous sendable chooser

    configureAutoCommands();

    if (COMMAND_LOGGING) {
      CommandScheduler.getInstance().onCommandInitialize(
          command -> Shuffleboard.addEventMarker("Command initialized",
              command.getClass().getName().substring(command.getClass().getName().lastIndexOf('.') + 1),
              EventImportance.kNormal));

      CommandScheduler.getInstance().onCommandInterrupt(
          command -> Shuffleboard.addEventMarker("Command interrupted",
              command.getClass().getName().substring(command.getClass().getName().lastIndexOf('.') + 1),
              EventImportance.kNormal));

      CommandScheduler.getInstance().onCommandFinish(
          command -> Shuffleboard.addEventMarker("Command finished",
              command.getClass().getName().substring(command.getClass().getName().lastIndexOf('.') + 1),
              EventImportance.kNormal));
    }
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDrivetrainButtons();
    configureIntakeButtons();
    configureShooterButtons();
    configureClimberButtons();
  }

  private void configureDrivetrainButtons() {
    joystickButtons1[3].whileHeld(new InstantCommand(() -> m_drivetrainSubsystem.setXStance(), m_drivetrainSubsystem));

    joystickButtons0[3].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> m_drivetrainSubsystem.disableFieldRelative(), m_drivetrainSubsystem),
            new InstantCommand(() -> m_drivetrainSubsystem.enableFieldRelative(), m_drivetrainSubsystem),
            m_drivetrainSubsystem::getFieldRelative));

    joystickButtons1[4]
        .whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setCenterGrav(0, 0), m_drivetrainSubsystem));

    // Reset Gyro
    xboxButtons[BUTTON_Y].whenPressed(
        new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope(), m_drivetrainSubsystem));
  }

  private void configureIntakeButtons() {
    // Change Colletor state

    operatorButtons[11].toggleWhenPressed( // FIXME: update software feature sheet if this is correct
        new ConditionalCommand(
            new InstantCommand(() -> m_collector.disableCollector(), m_collector),
            new InstantCommand(() -> m_collector.enableCollector(), m_collector),
            m_collector::isEnabled));

    // intake
    joystickButtons1[1].toggleWhenPressed(
        new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_collector.disableCollector(), m_collector),
                new InstantCommand(() -> m_storage.disableStorage(), m_storage)),
            new SequentialCommandGroup(
                new InstantCommand(() -> m_collector.enableCollector(), m_collector),
                new SortStorageCommand(m_storage),
                new InstantCommand(() -> m_collector.disableCollector(), m_collector)),
            m_collector::isEnabled));

    // unjam all
    xboxButtons[BUTTON_X].whenHeld(
        new ParallelCommandGroup(
            new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), m_collector),
            new InstantCommand(() -> m_storage.setStoragePower(StorageConstants.OUTTAKE_POWER), m_storage)));
    xboxButtons[BUTTON_X].whenReleased(
        new ParallelCommandGroup(
            new InstantCommand(() -> m_collector.setCollectorPower(0), m_collector),
            new InstantCommand(() -> m_storage.setStoragePower(0), m_storage)));

    // unjam collector
    xboxButtons[BUTTON_B].whenHeld(
        new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), m_collector));

    xboxButtons[BUTTON_B].whenReleased(
        new InstantCommand(() -> m_collector.setCollectorPower(0), m_collector));
  }

  private void configureShooterButtons() {

  }

  private void configureClimberButtons() {

    // configure climb to fourth rung climb sequence
    operatorButtons[8].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(m_elevator),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
            new ReachToNextRungCommand(m_elevator),
            new ParallelCommandGroup(
                new RetractClimberFullCommand(m_elevator),
                new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut())),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
            new ReachToNextRungCommand(m_elevator),
            new RetractClimberMinimumCommand(m_elevator)));

    // configure climb to third rung climb sequence
    operatorButtons[7].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(m_elevator),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut()),
            new ReachToNextRungCommand(m_elevator),
            new ParallelCommandGroup(
                new RetractClimberMinimumCommand(m_elevator),
                new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()))));

    // configure climb to 2 rung climb sequence
    operatorButtons[1].whenPressed(
        new RetractClimberMinimumCommand(m_elevator));

    // configure raise elevator before starting climb
    operatorButtons[2].whenPressed(
        new ExtendClimberToMidRungCommand(m_elevator));

    // FIXME change the following few commands to xbox buttons after merged to
    // toggle secondary arm override
    // operatorButtons[0].toggleWhenPressed(
    // new ConditionalCommand(
    // new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut(),
    // m_secondMechanism),
    // new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn(),
    // m_secondMechanism),
    // m_secondMechanism::isIn));

    // elevator up override
    // FIXME: how to map to d-pad?
    // operatorButtons[0]
    // .whileHeld(new InstantCommand(() ->
    // m_elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER),
    // m_elevator));

    // elevator down override
    // FIXME: how to map to d-pad?
    // operatorButtons[0]
    // .whileHeld(new InstantCommand(() ->
    // m_elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER *
    // -1), m_elevator));

    // resetElevator
    // FIXME: how to map to d-pad?
    // operatorButtons[0].whenPressed(new RetractClimberFullCommand(m_elevator));

    // climber emergency pause

    // FIXME this should be changed to start buttons and pass in the back button
    // operatorButtons[0].whenPressed(new InstantCommand(() ->
    // m_elevator.elevatorPause(operatorButtons[0].get(), m_elevator)));

    operatorButtons[12].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> m_elevator.disableElevatorControl(), m_elevator),
            new InstantCommand(() -> m_elevator.enableElevatorControl(), m_elevator),
            m_elevator::isElevatorControlEnabled));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private void configureAutoCommands() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    autoLeaveTarmac = new FollowPath(PathPlanner.loadPath("forward",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
        thetaController, m_drivetrainSubsystem);

    autoBlue1 = new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.enableCollector(), m_collector),
        // new SortStorageCommand(m_storage),
        new FollowPath(PathPlanner.loadPath("Blue1(1)",
            AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            thetaController, m_drivetrainSubsystem));
    // new WaitCommand(5)
    // new FollowPath(PathPlanner.loadPath("Blue1(2)",
    // AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared),
    // thetaController, m_drivetrainSubsystem)
    // add shoot from fender command

    autoRed1 = new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.enableCollector(), m_collector),
        // new SortStorageCommand(m_storage),
        new FollowPath(PathPlanner.loadPath("Red1(1)",
            AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            thetaController, m_drivetrainSubsystem));
    // new WaitCommand(5)
    // new FollowPath(PathPlanner.loadPath("Red1(2)",
    // AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared),
    // thetaController, m_drivetrainSubsystem)
    // add shoot from fender command

    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    m_chooser.addOption("Leave Tarmac", autoLeaveTarmac);
    m_chooser.addOption("Blue 1", autoBlue1);
    m_chooser.addOption("Red 1", autoRed1);
    tab.add("Auto Mode", m_chooser);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
