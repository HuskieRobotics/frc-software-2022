// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondaryArm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.StorageConstants;
import static frc.robot.Constants.JoystickConstants.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.GenericHID;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.DrivetrainSubsystem;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
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

// Joysticks

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    
  this.joystickButtons0 = new JoystickButton[13];
  this.joystickButtons1 = new JoystickButton[13];
  this.operatorButtons = new JoystickButton[13];
  this.xboxButtons = new JoystickButton[10];
  for(int i = 1; i < joystickButtons0.length; i++) {
      joystickButtons0[i] = new JoystickButton(joystick0, i);
      joystickButtons1[i] = new JoystickButton(joystick1, i);
      operatorButtons[i] = new JoystickButton(operatorConsole, i);
  }

  for(int i = 1; i < xboxButtons.length; i++){
    xboxButtons[i-1] = new JoystickButton(xboxController, i);
  }
    m_drivetrainSubsystem.register();
    m_storage.register();
    m_collector.register();
    m_flywheel.register();
    m_hood.register();
    m_limelight.register();
    m_storage.register();
    m_elevator.register();
    //m_secondMechanism.register();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(joystick0.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystick0.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(joystick1.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * -1));
        

      
    
    
    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
       
        
        // // configure climb to fourth rung climb sequence 
        // consoleButtons[2].whenPressed(
        //     new SequentialCommandGroup(
        //       new RetractClimberFullCommand(m_elevator),
        //       new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
        //       new ReachToNextRungCommand(m_elevator),
        //       new ParallelCommandGroup(
        //         new RetractClimberFullCommand(m_elevator),
        //         new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut())
        //       ),
        //       new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
        //       new ReachToNextRungCommand(m_elevator),
        //       new RetractClimberMinimumCommand(m_elevator)
        //     )
        // );

        // //configure climb to third rung climb sequence 
        // consoleButtons[11].whenPressed(
        //     new SequentialCommandGroup(
        //       new RetractClimberFullCommand(m_elevator),
        //       new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut()),
        //       new ReachToNextRungCommand(m_elevator),
        //       new ParallelCommandGroup(
        //         new RetractClimberMinimumCommand(m_elevator),
        //         new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()))));

        // //configure climb to 2 rung climb sequence 
        // consoleButtons[10].whenPressed(
        //   new RetractClimberMinimumCommand(m_elevator));

        // //configure raise elevator before starting climb
        // consoleButtons[9].whenPressed(  
        //   new ExtendClimberToMidRungCommand(m_elevator));
        
        // //FIXME change the following few commands to xbox buttons after merged to main
        // //toggle secondary arm override
        // consoleButtons[0].toggleWhenPressed(
        //   new ConditionalCommand(
        //     new InstantCommand(() -> m_secondMechanism.moveSecondaryArmOut()), 
        //     new InstantCommand(() -> m_secondMechanism.moveSecondaryArmIn()),
        //     m_secondMechanism :: isIn));
  joystickButtons1[3].whileHeld(new InstantCommand(() -> m_drivetrainSubsystem.setXStance()));
  joystickButtons0[3].toggleWhenPressed(
    new ConditionalCommand(
      new InstantCommand(() -> m_drivetrainSubsystem.disableFieldRelative()),
      new InstantCommand(() -> m_drivetrainSubsystem.enableFieldRelative()),
      m_drivetrainSubsystem::getFieldRelative));
    
    //change to use whenHeld(m_drivetrainSubsystem::setCenterGrav(0,0));
    //create command?
    joystickButtons1[4].whenHeld(new InstantCommand(() -> m_drivetrainSubsystem.setCenterGrav(0, 0), m_drivetrainSubsystem));
    
      



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    
    //Change Colletor state
    
    operatorButtons[11].toggleWhenPressed(
      new ConditionalCommand(
        new InstantCommand(() -> m_collector.disableCollector()),
        new InstantCommand(() -> m_collector.enableCollector()),
        m_collector :: isEnabled
    ));
    
    
    //intake
      joystickButtons1[1].toggleWhenPressed(
        new ConditionalCommand(
          new ParallelCommandGroup(
            new InstantCommand(() -> m_collector.disableCollector()),
            new InstantCommand(() -> m_storage.disableStorage())),
          new SequentialCommandGroup(
              new InstantCommand(() -> m_collector.enableCollector()),
              new SortStorageCommand(m_storage),
              new InstantCommand(() -> m_collector.disableCollector())),
            m_collector::isEnabled));
              
    //unjam all
    xboxButtons[BUTTON_X].whenHeld(
      new ParallelCommandGroup(
        new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER)),
        new InstantCommand(() -> m_storage.setStoragePower(StorageConstants.OUTTAKE_POWER))
      )
    );
    xboxButtons[BUTTON_X].whenReleased(
      new ParallelCommandGroup(
        new InstantCommand(() -> m_collector.setCollectorPower(0)),
        new InstantCommand(() -> m_storage.setStoragePower(0))
      )
    );

    //unjam collector
    xboxButtons[BUTTON_B].whenHeld(
        new InstantCommand(() -> m_collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER)));

    xboxButtons[BUTTON_B].whenReleased(
        new InstantCommand(() -> m_collector.setCollectorPower(0)));

      //Reset statemachine Button A

      //Reset Gyro
      xboxButtons[BUTTON_Y].whenPressed(
        new InstantCommand(()-> m_drivetrainSubsystem.zeroGyroscope())
      );
  }


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

        // //elevator up override
        // consoleButtons[0].whileHeld(new InstantCommand(() -> m_elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER)));

        // //elevator down override
        // consoleButtons[0].whileHeld(new InstantCommand(() -> m_elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER * -1)));

        // //resetElevator 
        // consoleButtons[0].whenPressed(new RetractClimberFullCommand(m_elevator));
  
// //climber emergency pause

        // //FIXME this should be changed to start buttons and pass in the back button
        // consoleButtons[0].whenPressed(new InstantCommand(() -> m_elevator.elevatorPause(consoleButtons[0].get())));

        // consoleButtons[0].toggleWhenPressed(
        //   new ConditionalCommand(
        //     new InstantCommand(() -> m_elevator.disableElevatorControl()),
        //     new InstantCommand(() -> m_elevator.enableElevatorControl()),
        //     m_elevator :: isElevatorControlEnabled));

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {

    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    

    // The selected command will be run in autonomous
    return new FollowPath(PathPlanner.loadPath("curveTest",
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared), thetaController, m_drivetrainSubsystem);
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

