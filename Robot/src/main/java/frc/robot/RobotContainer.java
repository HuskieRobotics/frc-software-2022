package frc.robot;

import static frc.robot.Constants.JoystickConstants.*;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flywheel;
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
  //private final Hood m_hood = new Hood();
  //private final LimelightMath m_limelight = new LimelightMath();
  private final SecondaryArm m_secondMechanism = new SecondaryArm();
  private final Elevator m_elevator = new Elevator();

  private Command autoBlueForward;
  private Command autoBlue1;
  private Command autoBlue2;
  private Command autoBlue3;
  private Command autoBlue4;

  // Joysticks

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    LiveWindow.disableAllTelemetry();

    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];
    this.operatorButtons = new JoystickButton[13];
    this.xboxButtons = new JoystickButton[11];
    for (int i = 1; i < joystickButtons0.length; i++) {
      joystickButtons0[i] = new JoystickButton(joystick0, i);
      joystickButtons1[i] = new JoystickButton(joystick1, i);
      operatorButtons[i] = new JoystickButton(operatorConsole, i);
    }

    for (int i = 1; i < xboxButtons.length; i++) {
      xboxButtons[i] = new JoystickButton(xboxController, i);
    }

    m_drivetrainSubsystem.register();
    m_storage.register();
    m_collector.register();
    m_flywheel.register();
    //m_hood.register();
    //m_limelight.register();
    m_storage.register();
    m_elevator.register();
     m_secondMechanism.register();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystick1.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Smartdashboard Subsystems

    // SmartDashboard Buttons

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // Configure autonomous sendable chooser

    configureAutoCommands();

    if(TUNING) {
      Shuffleboard.getTab("Elevator").add("Reach to Next Rung", new ReachToNextRungCommand(m_elevator, m_secondMechanism));
      Shuffleboard.getTab("Elevator").add("Extend Before Next", new ExtendClimberBeforeNextRungCommand(m_elevator, m_secondMechanism));
    }
            

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
    //x-stance
    joystickButtons1[3].whileHeld(new InstantCommand(() -> m_drivetrainSubsystem.setXStance(), m_drivetrainSubsystem));

    //FieldRelative toggle
    joystickButtons0[3].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> m_drivetrainSubsystem.disableFieldRelative(), m_drivetrainSubsystem),
            new InstantCommand(() -> m_drivetrainSubsystem.enableFieldRelative(), m_drivetrainSubsystem),
            m_drivetrainSubsystem::getFieldRelative));

    //center of gravity
    joystickButtons1[4].whenPressed(
          new InstantCommand(() -> 
              m_drivetrainSubsystem.rotateEvasively(-modifyAxis(joystick0.getY()), -modifyAxis(joystick0.getX()), -modifyAxis(joystick1.getX())),
              m_drivetrainSubsystem));
    joystickButtons1[4]
        .whenReleased(new InstantCommand(() -> m_drivetrainSubsystem.resetCenterGrav(), m_drivetrainSubsystem));

    joystickButtons0[1].whenPressed(
      new ParallelCommandGroup(
          new InstantCommand(() -> m_flywheel.stopFlywheel(), m_flywheel),
          new InstantCommand(()-> m_storage.disableStorage(), m_storage),
          new InstantCommand(() -> m_collector.disableCollector(), m_collector),
          new InstantCommand(() -> m_drivetrainSubsystem.disableXstance(), m_drivetrainSubsystem)));

    // Reset Gyro
    xboxButtons[BUTTON_Y].whenPressed(
        new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope(), m_drivetrainSubsystem));
  }

  private void configureIntakeButtons() {
    //toggle collector
    xboxButtons[JoystickConstants.LEFT_JOYSTICK_BUTTON].toggleWhenPressed( 
        new ConditionalCommand(
            new InstantCommand(() -> m_collector.disableCollector(), m_collector),
            new InstantCommand(() -> m_collector.enableCollector(), m_collector),
            m_collector::isEnabled));

    //toggle storage
    xboxButtons[JoystickConstants.RIGHT_JOYSTICK_BUTTON].toggleWhenPressed( 
        new ConditionalCommand(
            new InstantCommand(() -> m_storage.disableStorage(), m_storage),
            new InstantCommand(() -> m_storage.enableStorage(), m_storage),
            m_storage :: isStorageEnabled));

    // intake
    joystickButtons1[1].toggleWhenPressed( 
        new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_collector.disableCollector(), m_collector),
                new InstantCommand(() -> m_storage.disableStorage(), m_storage),
                new InstantCommand(() -> m_flywheel.stopFlywheel(), m_flywheel)),
            new SequentialCommandGroup(
              new InstantCommand(() -> m_collector.enableCollector(), m_collector),
              new SortStorageCommand(m_storage),
              new InstantCommand(() -> m_collector.disableCollector(), m_collector),
              new SetFlywheelVelocityCommand(m_flywheel, FlywheelConstants.WALL_SHOT_VELOCITY)),
            m_collector::isEnabled));

    // unjam all
    xboxButtons[BUTTON_X].whenHeld(
        new ParallelCommandGroup(
            new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), m_collector),
            new InstantCommand(() -> m_storage.setStoragePower(StorageConstants.OUTTAKE_POWER), m_storage),
            new InstantCommand(() -> m_flywheel.unjamFlywheel(), m_flywheel)
            ));
    xboxButtons[BUTTON_X].whenReleased(
        new ParallelCommandGroup(
            new InstantCommand(() -> m_collector.setCollectorPower(0), m_collector),
            new InstantCommand(() -> m_storage.setStoragePower(0), m_storage),
            new InstantCommand(() -> m_flywheel.stopFlywheel(), m_flywheel)
            ));

    // unjam collector
    xboxButtons[BUTTON_B].whenHeld(
        new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), m_collector));

    xboxButtons[BUTTON_B].whenReleased(
        new InstantCommand(() -> m_collector.setCollectorPower(0), m_collector));
  }

  private void configureShooterButtons() {

    // enable/disable limelight aiming
    operatorButtons[JoystickConstants.LIMELIGHT_AIM_TOGGLE].toggleWhenPressed( 
        new ConditionalCommand(
            new InstantCommand(() -> m_drivetrainSubsystem.disableLimelightAim(), m_drivetrainSubsystem),
            new InstantCommand(() -> m_drivetrainSubsystem.enableLimelightAim(), m_drivetrainSubsystem),
            m_drivetrainSubsystem::isLimelightAimEnabled));

    //preset field wall
    operatorButtons[JoystickConstants.FIELD_WALL].whenPressed(
      createShootCommandSequence(DrivetrainConstants.LIMELIGHT_ALIGNMENT_TOLERANCE, FlywheelConstants.WALL_SHOT_VELOCITY));

    //preset launchpad
    operatorButtons[JoystickConstants.LAUNCHPAD].whenPressed(
      createShootCommandSequence(DrivetrainConstants.LIMELIGHT_LAUNCHPAD_ALIGNMENT_TOLERANCE, FlywheelConstants.LAUNCH_PAD_VELOCITY));
        
    //shoot slow
    operatorButtons[JoystickConstants.SHOOT_SLOW].whenPressed(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_collector.disableCollector(), m_collector),
          new SetFlywheelVelocityCommand(m_flywheel, FlywheelConstants.SHOOT_SLOW_VELOCITY)),
        new InstantCommand(()-> m_storage.enableStorage(), m_storage),
        new WaitForShotCommand(m_storage, m_flywheel, m_drivetrainSubsystem)));
    
    operatorButtons[JoystickConstants.SHOOT_LIMELIGHT].whenPressed(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_collector.disableCollector(), m_collector),
          new LimelightSetFlywheelVelocityCommand(m_flywheel, m_drivetrainSubsystem),
          new SequentialCommandGroup (
            new LimelightAlignToTargetCommand(DrivetrainConstants.LIMELIGHT_ALIGNMENT_TOLERANCE, m_drivetrainSubsystem),
            new InstantCommand(()-> m_drivetrainSubsystem.enableXstance(), m_drivetrainSubsystem))),
        new InstantCommand(()-> m_storage.enableStorage(), m_storage),
        new WaitForShotCommand(m_storage, m_flywheel, m_drivetrainSubsystem)));
  }

  private void configureClimberButtons() {

    // configure climb to 4 (traverse) rung climb sequence
    operatorButtons[8].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(m_elevator),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut(), m_secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungCommand(m_elevator, m_secondMechanism),
            new ParallelCommandGroup(
                new RetractClimberMinimumCommand(m_elevator),
                new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn(), m_secondMechanism)),
            new RetractClimberFullCommand(m_elevator),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut(), m_secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(m_elevator, m_secondMechanism),
            new RetractClimberMinimumWithPitchCommand(m_elevator)));

    // configure climb to 3 (high) rung climb sequence
    operatorButtons[7].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(m_elevator),
            new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut(), m_secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(m_elevator, m_secondMechanism),
            new RetractClimberMinimumWithPitchCommand(m_elevator)));

    // configure climb to 1/2 (low/mid) rung climb sequence
    operatorButtons[1].whenPressed(
        new SequentialCommandGroup(
          new RetractClimberFullCommand(m_elevator),
          new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut(), m_secondMechanism)));

    // configure raise elevator before starting climb to 1 (low) rung; FIXME: confirm button
    operatorButtons[11].whenPressed(
        new ExtendClimberToLowRungCommand(m_elevator));

    // configure raise elevator before starting climb
    operatorButtons[2].whenPressed(
        new ExtendClimberToMidRungCommand(m_elevator));
    
    //reset climber
    xboxButtons[BUTTON_A].whenPressed(
      new RetractClimberFullCommand(m_elevator)
    );

    //extend climber
    xboxButtons[JoystickConstants.BUTTON_RB].whenPressed(
      new InstantCommand(() -> m_elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER), m_elevator));


    xboxButtons[JoystickConstants.BUTTON_RB].whenReleased(
      new InstantCommand(() -> m_elevator.setElevatorMotorPower(0), m_elevator));

    //retract climber
    xboxButtons[JoystickConstants.BUTTON_LB].whenPressed(
      new InstantCommand(() -> m_elevator.setElevatorMotorPower(-1 * ElevatorConstants.DEFAULT_MOTOR_POWER), m_elevator));

    xboxButtons[JoystickConstants.BUTTON_LB].whenReleased(
      new InstantCommand(() -> m_elevator.setElevatorMotorPower(0), m_elevator));

    //pause elevator
    xboxButtons[JoystickConstants.BUTTON_START].whenPressed(new InstantCommand(() ->
        m_elevator.elevatorPause(xboxButtons[JoystickConstants.BUTTON_BACK].get()), m_elevator));

    operatorButtons[12].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> m_elevator.disableElevatorControl(), m_elevator),
            new InstantCommand(() -> m_elevator.enableElevatorControl(), m_elevator),
            m_elevator::isElevatorControlEnabled));
  
    operatorButtons[JoystickConstants.SECONDARY].toggleWhenPressed(
        new ConditionalCommand(
          new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut()),
          new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
          m_secondMechanism::isIn));
        
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  // always start teleop in a known state and ready to drive
  public void teleopInit() {
    m_storage.disableStorage();
    m_flywheel.stopFlywheel();
    m_collector.disableCollector();
  }

  private void configureAutoCommands() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory autoBlueForwardPath = PathPlanner.loadPath("BlueForward",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    autoBlueForward = new SequentialCommandGroup(
      new FollowPath(autoBlueForwardPath, thetaController, m_drivetrainSubsystem, true),
      createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

      PathPlannerTrajectory autoBlue1Path = PathPlanner.loadPath("Blue1(1)",
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      autoBlue1 = new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.enableCollector(), m_collector),
        new FollowPath(autoBlue1Path, thetaController, m_drivetrainSubsystem, true),
        new InstantCommand(() -> m_collector.disableCollector(), m_collector),
        createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

      PathPlannerTrajectory autoBlue2Path = PathPlanner.loadPath("Blue2(1)",
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      autoBlue2 = new SequentialCommandGroup(
        new InstantCommand(() -> m_collector.enableCollector(), m_collector),
        new FollowPath(autoBlue2Path, thetaController, m_drivetrainSubsystem, true),
        new InstantCommand(() -> m_collector.disableCollector(), m_collector),
        createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

      PathPlannerTrajectory autoBlue31Path = PathPlanner.loadPath("Blue3(1)",
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory autoBlue32Path = PathPlanner.loadPath("Blue3(2)",
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory autoBlue33Path = PathPlanner.loadPath("Blue3(3)",
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      autoBlue3 = new SequentialCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_collector.enableCollector(), m_collector),
          new InstantCommand(() -> m_flywheel.setVelocity(FlywheelConstants.WALL_SHOT_VELOCITY), m_flywheel)),
        new WaitCommand(0.5),
        new FollowPath(autoBlue31Path, thetaController, m_drivetrainSubsystem, true),
        limelightCreateAutoShootCommandSequence(2),
        new ParallelDeadlineGroup(
          new FollowPath(autoBlue32Path, thetaController, m_drivetrainSubsystem, false),
          new SortStorageCommand(m_storage)),
        new ParallelCommandGroup(
            new InstantCommand(() -> m_flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), m_flywheel),
            new FollowPath(autoBlue33Path, thetaController, m_drivetrainSubsystem, false)),
        limelightCreateAutoShootCommandSequence(15));

      PathPlannerTrajectory autoBlue41Path = PathPlanner.loadPath("Blue4(1)",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory autoBlue42Path = PathPlanner.loadPath("Blue4(2)",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory autoBlue43Path = PathPlanner.loadPath("Blue4(3)",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory autoBlue44Path = PathPlanner.loadPath("Blue4(4)",
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      autoBlue4 = new SequentialCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_collector.enableCollector(), m_collector),
          new InstantCommand(() -> m_flywheel.setVelocity(FlywheelConstants.WALL_SHOT_VELOCITY), m_flywheel)),
        new WaitCommand(0.5),
        new FollowPath(autoBlue41Path, thetaController, m_drivetrainSubsystem, true),
        limelightCreateAutoShootCommandSequence(2),
        new ParallelDeadlineGroup(
          new FollowPath(autoBlue42Path, thetaController, m_drivetrainSubsystem, false),
          new SortStorageCommand(m_storage)),
        limelightCreateAutoShootCommandSequence(2),
        new FollowPath(autoBlue43Path, thetaController, m_drivetrainSubsystem, false),
        new InstantCommand(() -> m_drivetrainSubsystem.stop()),
        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new SortStorageCommand(m_storage)),
        new ParallelCommandGroup(
            new InstantCommand(() -> m_flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), m_flywheel),
            new FollowPath(autoBlue44Path, thetaController, m_drivetrainSubsystem, false)),
        limelightCreateAutoShootCommandSequence(5));
  

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    m_chooser.addOption("Blue Forward", autoBlueForward);
    m_chooser.addOption("Blue 1", autoBlue1);
    m_chooser.addOption("Blue 2", autoBlue2);
    m_chooser.addOption("Blue 3", autoBlue3);
    m_chooser.addOption("Blue 4", autoBlue4);
    tab.add("Auto Mode", m_chooser);
  }

  private Command createShootCommandSequence(double aimTolerance, int shotVelocity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
          new InstantCommand(() -> m_collector.disableCollector(), m_collector),
          new SetFlywheelVelocityCommand(m_flywheel, shotVelocity),
          new SequentialCommandGroup (
            new LimelightAlignToTargetCommand(aimTolerance, m_drivetrainSubsystem),
            new InstantCommand(()-> m_drivetrainSubsystem.enableXstance(), m_drivetrainSubsystem))),
        new InstantCommand(()-> m_storage.enableStorage(), m_storage),
        new WaitForShotCommand(m_storage, m_flywheel, m_drivetrainSubsystem));
  }

  private Command createAutoShootCommandSequence(int shotVelocity, double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
          new SetFlywheelVelocityCommand(m_flywheel, shotVelocity),
          new LimelightAlignToTargetCommand(DrivetrainConstants.LIMELIGHT_ALIGNMENT_TOLERANCE, m_drivetrainSubsystem)),
        new InstantCommand(()-> m_storage.enableStorage(), m_storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
          new InstantCommand(() -> m_flywheel.stopFlywheel(), m_flywheel),
          new InstantCommand(()-> m_storage.disableStorage(), m_storage)));
  }

  private Command limelightCreateAutoShootCommandSequence(double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
          new LimelightSetFlywheelVelocityCommand(m_flywheel, m_drivetrainSubsystem),
          new LimelightAlignToTargetCommand(DrivetrainConstants.LIMELIGHT_ALIGNMENT_TOLERANCE, m_drivetrainSubsystem)),
        new InstantCommand(()-> m_storage.enableStorage(), m_storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
          new InstantCommand(() -> m_flywheel.stopFlywheel(), m_flywheel),
          new InstantCommand(()-> m_storage.disableStorage(), m_storage)));
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

  public boolean isElevatorControlEnabled() {
    return m_elevator.isElevatorControlEnabled();
  }
}
