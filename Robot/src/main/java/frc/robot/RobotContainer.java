package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.JoystickConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
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

  private static RobotContainer robotContainer = new RobotContainer();

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Storage storage = new Storage();
  private final Collector collector = new Collector();
  private final Flywheel flywheel = new Flywheel();
  // private final Hood m_hood = new Hood();
  // private final LimelightMath m_limelight = new LimelightMath();
  private final SecondaryArm secondMechanism = new SecondaryArm();
  private final Elevator elevator = new Elevator();

  private Command autoOneBall;
  private Command autoTwoBallSteal;
  private Command autoTwoBallAlt;
  private Command autoFiveBall;
  private Command autoFiveBallAlt;
  private Command autoTwoBall;

  // Joysticks

  // A chooser for autonomous commands
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  private RobotContainer() {

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];
    this.operatorButtons = new JoystickButton[13];
    this.xboxButtons = new JoystickButton[11];

    // buttons use 1-based indexing such that the index matches the button number
    for (int i = 1; i < joystickButtons0.length; i++) {
      joystickButtons0[i] = new JoystickButton(joystick0, i);
      joystickButtons1[i] = new JoystickButton(joystick1, i);
      operatorButtons[i] = new JoystickButton(operatorConsole, i);
    }

    for (int i = 1; i < xboxButtons.length; i++) {
      xboxButtons[i] = new JoystickButton(xboxController, i);
    }

    // all subsystems must register with the Command Scheduler in order for their periodic methods
    //  to be invoked
    drivetrainSubsystem.register();
    storage.register();
    collector.register();
    flywheel.register();
    // m_hood.register();
    // m_limelight.register();
    storage.register();
    elevator.register();
    secondMechanism.register();

    // Set up the default command for the drivetrain.
    // The joysticks' values map to percentage of the maximum velocities.
    // The velocities may be specified from either the robot's frame of reference or the field's
    //  frame of reference. In the robot's frame of reference, the positive x direction is forward;
    //  the positive y direction, left; position rotation, CCW. In the field frame of reference,
    //  the origin of the field to the lower left corner (i.e., the corner of the field to the
    //  driver's right). Zero degrees is away from the driver and increases in the CCW direction.
    // This is why the left joystick's y axis specifies the velocity in the x direction and the
    //  left joystick's x axis specifies the velocity in the y direction.
    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () ->
                -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(joystick1.getX())
                    * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Smartdashboard Subsystems

    // SmartDashboard Buttons

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // Configure autonomous sendable chooser

    configureAutoCommands();

    if (TUNING) {
      Shuffleboard.getTab("Elevator")
          .add("Reach to Next Rung", new ReachToNextRungCommand(elevator, secondMechanism));
    }

    if (COMMAND_LOGGING) {
      CommandScheduler.getInstance()
          .onCommandInitialize(
              command ->
                  Shuffleboard.addEventMarker(
                      "Command initialized",
                      command
                          .getClass()
                          .getName()
                          .substring(command.getClass().getName().lastIndexOf('.') + 1),
                      EventImportance.kNormal));

      CommandScheduler.getInstance()
          .onCommandInterrupt(
              command ->
                  Shuffleboard.addEventMarker(
                      "Command interrupted",
                      command
                          .getClass()
                          .getName()
                          .substring(command.getClass().getName().lastIndexOf('.') + 1),
                      EventImportance.kNormal));

      CommandScheduler.getInstance()
          .onCommandFinish(
              command ->
                  Shuffleboard.addEventMarker(
                      "Command finished",
                      command
                          .getClass()
                          .getName()
                          .substring(command.getClass().getName().lastIndexOf('.') + 1),
                      EventImportance.kNormal));
    }
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDrivetrainButtons();
    configureIntakeButtons();
    configureShooterButtons();
    configureClimberButtons();
  }

  private void configureDrivetrainButtons() {
    // auto aim and shoot while moving
    operatorButtons[JoystickConstants.AUTO_AIM_AND_SHOOT].whenPressed(
        new SequentialCommandGroup(
            new IndexSingleBallCommand(storage),
            new LimelightAlignOnMoveCommand(
                drivetrainSubsystem, flywheel, collector, joystick0, joystick1),
            new WaitCommand(0.300),
            createLimelightShootCommandSequence(true /* use gyro */)));

    // FieldRelative toggle
    joystickButtons0[3].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(
                () -> drivetrainSubsystem.disableFieldRelative(), drivetrainSubsystem),
            new InstantCommand(
                () -> drivetrainSubsystem.enableFieldRelative(), drivetrainSubsystem),
            drivetrainSubsystem::getFieldRelative));

    // center of gravity
    joystickButtons1[4].whenPressed(
        new InstantCommand(
            () ->
                drivetrainSubsystem.rotateEvasively(
                    -modifyAxis(joystick0.getY()),
                    -modifyAxis(joystick0.getX()),
                    -modifyAxis(joystick1.getX())),
            drivetrainSubsystem));
    joystickButtons1[4].whenReleased(
        new InstantCommand(() -> drivetrainSubsystem.resetCenterGrav(), drivetrainSubsystem));
    // reset all(2)
    joystickButtons1[3].whenPressed(
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage),
            new InstantCommand(() -> collector.disableCollector(), collector),
            new InstantCommand(() -> drivetrainSubsystem.disableXstance(), drivetrainSubsystem)));

    // Reset Gyro
    xboxButtons[BUTTON_Y].whenPressed(
        new InstantCommand(() -> drivetrainSubsystem.zeroGyroscope(), drivetrainSubsystem));
  }

  private void configureIntakeButtons() {
    // toggle collector
    xboxButtons[JoystickConstants.LEFT_JOYSTICK_BUTTON].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new InstantCommand(() -> collector.enableCollector(), collector),
            collector::isEnabled));

    // toggle storage
    xboxButtons[JoystickConstants.RIGHT_JOYSTICK_BUTTON].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> storage.disableStorage(), storage),
            new InstantCommand(() -> storage.enableStorage(), storage),
            storage::isStorageEnabled));

    // intake
    joystickButtons1[1].toggleWhenPressed(
        new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.disableCollector(), collector),
                new InstantCommand(() -> storage.disableStorage(), storage),
                new InstantCommand(() -> flywheel.stopFlywheel(), flywheel)),
            new SequentialCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new SortStorageCommand(storage),
                new InstantCommand(() -> collector.disableCollector(), collector),
                new SetFlywheelVelocityCommand(flywheel, FlywheelConstants.WALL_SHOT_VELOCITY)),
            collector::isEnabled));

    // unjam all
    xboxButtons[BUTTON_X].whenHeld(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), collector),
            new InstantCommand(
                () -> storage.setStoragePower(StorageConstants.OUTTAKE_POWER), storage),
            new InstantCommand(() -> flywheel.unjamFlywheel(), flywheel)));
    xboxButtons[BUTTON_X].whenReleased(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.setCollectorPower(0), collector),
            new InstantCommand(() -> storage.setStoragePower(0), storage),
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel)));

    // unjam collector
    xboxButtons[BUTTON_B].whenHeld(
        new InstantCommand(
            () -> collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), collector));

    xboxButtons[BUTTON_B].whenReleased(
        new InstantCommand(() -> collector.setCollectorPower(0), collector));
  }

  private void configureShooterButtons() {

    // enable/disable limelight aiming
    operatorButtons[JoystickConstants.LIMELIGHT_AIM_TOGGLE].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(
                () -> drivetrainSubsystem.disableLimelightAim(), drivetrainSubsystem),
            new InstantCommand(() -> drivetrainSubsystem.enableLimelightAim(), drivetrainSubsystem),
            drivetrainSubsystem::isLimelightAimEnabled));

    // preset field wall
    operatorButtons[JoystickConstants.FIELD_WALL].whenPressed(
        createShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY));

    // preset launchpad
    operatorButtons[JoystickConstants.LAUNCHPAD].whenPressed(
        createShootCommandSequence(FlywheelConstants.LAUNCH_PAD_VELOCITY));

    // shoot slow
    operatorButtons[JoystickConstants.SHOOT_SLOW].whenPressed(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.disableCollector(), collector),
                new SetFlywheelVelocityCommand(flywheel, FlywheelConstants.SHOOT_SLOW_VELOCITY)),
            new InstantCommand(() -> storage.enableStorage(), storage),
            new WaitForShotCommand(storage, flywheel, drivetrainSubsystem)));

    // limelight shot(1)
    joystickButtons0[1].whenPressed(
        new SequentialCommandGroup(
            new ParallelCommandGroup(new IndexSingleBallCommand(storage), new WaitCommand(0.300)),
            createLimelightShootCommandSequence(true /* use gyro */)));
  }

  private void configureClimberButtons() {

    // configure climb to 4 (traverse) rung climb sequence
    operatorButtons[8].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(elevator, secondMechanism),
            new WaitCommand(
                ElevatorConstants
                    .RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG), // wait for secondary arm to be
            // positioned
            new RetractClimberMinimumCommand(
                ElevatorConstants.LATCH_TRAVERSE_RUNG_ENCODER_HEIGHT, elevator)));

    // configure climb to 3 (high) rung climb sequence
    operatorButtons[7].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(elevator, secondMechanism),
            new WaitCommand(
                ElevatorConstants
                    .RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG), // wait for secondary arm to be
            // positioned
            new RetractClimberMinimumCommand(
                ElevatorConstants.LATCH_HIGH_RUNG_ENCODER_HEIGHT, elevator)));

    // configure climb to 1/2 (low/mid) rung climb sequence
    operatorButtons[1].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism)));

    // configure raise elevator before starting climb to 1 (low) rung
    operatorButtons[11].whenPressed(new ExtendClimberToLowRungCommand(elevator));

    // configure raise elevator before starting climb
    operatorButtons[2].whenPressed(new ExtendClimberToMidRungCommand(elevator));

    // reset climber
    xboxButtons[BUTTON_A].whenPressed(new RetractClimberFullCommand(elevator));

    // extend climber
    xboxButtons[JoystickConstants.BUTTON_RB].whenPressed(
        new InstantCommand(
            () -> elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER), elevator));

    xboxButtons[JoystickConstants.BUTTON_RB].whenReleased(
        new InstantCommand(() -> elevator.setElevatorMotorPower(0), elevator));

    // retract climber
    xboxButtons[JoystickConstants.BUTTON_LB].whenPressed(
        new InstantCommand(
            () -> elevator.setElevatorMotorPower(-1 * ElevatorConstants.DEFAULT_MOTOR_POWER),
            elevator));

    xboxButtons[JoystickConstants.BUTTON_LB].whenReleased(
        new InstantCommand(() -> elevator.setElevatorMotorPower(0), elevator));

    // pause elevator
    xboxButtons[JoystickConstants.BUTTON_START].whenPressed(
        new InstantCommand(
            () -> elevator.elevatorPause(xboxButtons[JoystickConstants.BUTTON_BACK].get()),
            elevator));

    operatorButtons[12].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> elevator.disableElevatorControl(), elevator),
            new InstantCommand(() -> elevator.enableElevatorControl(), elevator),
            elevator::isElevatorControlEnabled));

    operatorButtons[JoystickConstants.SECONDARY].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut()),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmIn()),
            secondMechanism::isIn));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  // always start teleop in a known state and ready to drive
  public void teleopInit() {
    storage.disableStorage();
    flywheel.stopFlywheel();
    collector.disableCollector();
  }

  private void configureAutoCommands() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory autoBlue01Path =
        PathPlanner.loadPath(
            "Blue0(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue02Path =
        PathPlanner.loadPath(
            "Blue0(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    autoOneBall =
        new SequentialCommandGroup(
            new FollowPath(autoBlue01Path, thetaController, drivetrainSubsystem, true),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 2),
            new FollowPath(autoBlue02Path, thetaController, drivetrainSubsystem, false));

    PathPlannerTrajectory autoBlue11Path =
        PathPlanner.loadPath(
            "Blue1(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue12Path =
        PathPlanner.loadPath(
            "Blue1(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    autoTwoBallSteal =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue11Path, thetaController, drivetrainSubsystem, true),
            new WaitCommand(5.0),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue12Path, thetaController, drivetrainSubsystem, false),
                new SortStorageCommand(storage)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> collector.disableCollector(), collector),
                    new SetFlywheelVelocityCommand(
                        flywheel, FlywheelConstants.SHOOT_STEAL_VELOCITY)),
                new InstantCommand(() -> storage.enableStorage(), storage),
                new WaitCommand(15)));

    autoTwoBall =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue11Path, thetaController, drivetrainSubsystem, true),
            new WaitCommand(5.0),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

    PathPlannerTrajectory autoBlue2Path =
        PathPlanner.loadPath(
            "Blue2(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    autoTwoBallAlt =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue2Path, thetaController, drivetrainSubsystem, true),
            new InstantCommand(() -> collector.disableCollector(), collector),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

    // 5 ball auto (shoot 2; shoot 3 (with bowling))
    PathPlannerTrajectory autoBlue31Path =
        PathPlanner.loadPath(
            "Blue3(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue32Path =
        PathPlanner.loadPath(
            "Blue3(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue33Path =
        PathPlanner.loadPath(
            "Blue3(3)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    autoFiveBall =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.NEAR_WALL_SHOT_VELOCITY),
                    flywheel)),
            new WaitCommand(0.5),
            new FollowPath(autoBlue31Path, thetaController, drivetrainSubsystem, true),
            createAutoShootNoAimCommandSequence(FlywheelConstants.NEAR_WALL_SHOT_VELOCITY, 2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue32Path, thetaController, drivetrainSubsystem, false),
                new SortStorageCommand(storage)),
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), flywheel),
                new FollowPath(autoBlue33Path, thetaController, drivetrainSubsystem, false)),
            limelightCreateAutoShootCommandSequence(15));

    // 5-ball auto (shoot 2, shoot 1, shoot 2 (no bowling))
    PathPlannerTrajectory autoBlue41Path =
        PathPlanner.loadPath(
            "Blue4(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue42Path =
        PathPlanner.loadPath(
            "Blue4(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue43Path =
        PathPlanner.loadPath(
            "Blue4(3)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue44Path =
        PathPlanner.loadPath(
            "Blue4(4)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    autoFiveBallAlt =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.WALL_SHOT_VELOCITY), flywheel)),
            new WaitCommand(0.5),
            new FollowPath(autoBlue41Path, thetaController, drivetrainSubsystem, true),
            limelightCreateAutoShootCommandSequence(2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue42Path, thetaController, drivetrainSubsystem, false),
                new SortStorageCommand(storage)),
            limelightCreateAutoShootCommandSequence(2),
            new FollowPath(autoBlue43Path, thetaController, drivetrainSubsystem, false),
            new InstantCommand(() -> drivetrainSubsystem.stop()),
            new ParallelDeadlineGroup(new WaitCommand(2), new SortStorageCommand(storage)),
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), flywheel),
                new FollowPath(autoBlue44Path, thetaController, drivetrainSubsystem, false)),
            limelightCreateAutoShootCommandSequence(5));

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    chooser.addOption("1 Ball", autoOneBall);
    chooser.addOption("2 Ball & Steal", autoTwoBallSteal);
    chooser.addOption("2 Ball", autoTwoBall);
    chooser.addOption("Alt 2 Ball", autoTwoBallAlt);
    chooser.addOption("Main 5 Ball", autoFiveBall);
    chooser.addOption("Alt 5 Ball", autoFiveBallAlt);
    tab.add("Auto Mode", chooser);
  }

  private Command createShootCommandSequence(int shotVelocity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new SetFlywheelVelocityCommand(flywheel, shotVelocity),
            new SequentialCommandGroup(
                new LimelightAlignWithGyroCommand(drivetrainSubsystem),
                new InstantCommand(
                    () -> drivetrainSubsystem.enableXstance(), drivetrainSubsystem))),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitForShotCommand(storage, flywheel, drivetrainSubsystem));
  }

  private Command createLimelightShootCommandSequence(boolean useGyro) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new LimelightSetFlywheelVelocityCommand(flywheel, drivetrainSubsystem),
            new SequentialCommandGroup(
                (useGyro
                    ? new LimelightAlignWithGyroCommand(drivetrainSubsystem)
                    : new LimelightAlignToTargetCommand(drivetrainSubsystem)),
                new InstantCommand(
                    () -> drivetrainSubsystem.enableXstance(), drivetrainSubsystem))),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitForShotCommand(storage, flywheel, drivetrainSubsystem));
  }

  private Command createAutoShootCommandSequence(int shotVelocity, double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SetFlywheelVelocityCommand(flywheel, shotVelocity),
            new LimelightAlignWithGyroCommand(drivetrainSubsystem)),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
  }

  private Command createAutoShootNoAimCommandSequence(int shotVelocity, double shotDelay) {
    return new SequentialCommandGroup(
        new SetFlywheelVelocityCommand(flywheel, shotVelocity),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
  }

  private Command limelightCreateAutoShootCommandSequence(double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new LimelightSetFlywheelVelocityCommand(flywheel, drivetrainSubsystem),
            new LimelightAlignWithGyroCommand(drivetrainSubsystem)),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
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

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public boolean isElevatorControlEnabled() {
    return elevator.isElevatorControlEnabled();
  }

  // replace this method with one that better preserves encapsulation (e.g., disabledPeriodic)
  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
