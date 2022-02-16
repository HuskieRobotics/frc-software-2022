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

import frc.commands.ExtendClimberToHeightCommand;
// import frc.robot.commands.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SecondMechanism;

import javax.swing.ButtonGroup;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.command.StartCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Constants.SecondMechanismConstants;
import frc.robot.Constants.ElevatorConstants;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final JoystickButton[] consoleButtons;
  private final Joystick buttonConsole;

  private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final SecondMechanism m_secondMechanism = new SecondMechanism();
    public final Elevator m_elevator = new Elevator();

// Joysticks

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    
    this.buttonConsole = new Joystick(3);
    this.consoleButtons = new JoystickButton[11];
    for(int i = 1; i <= consoleButtons.length; i++) {
      consoleButtons[i-1] = new JoystickButton(buttonConsole, i);
    }
    
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems
    SmartDashboard.putData(m_secondMechanism);
    SmartDashboard.putData(m_elevator);


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
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
        
        // configure climb to fourth rung climb sequence 
        consoleButtons[2].whenPressed(
            new SequentialCommandGroup(
              new ExtendClimberToHeightCommand(m_elevator),
              new InstantCommand(() -> m_secondMechanism.setSecondMechanism(Constants.SecondMechanismConstants.SECOND_MECHANISM_PULL)),
              new ExtendClimberToHeightCommand(m_elevator, Constants.ElevatorConstants.THIRD_RUNG_ENCODER_HEIGHT),
              new ParallelCommandGroup(
                new ExtendClimberToHeightCommand(m_elevator),
                new InstantCommand(() -> m_secondMechanism.setSecondMechanism(Constants.SecondMechanismConstants.SECOND_MECHANISM_PUSH))
              ),
              new InstantCommand(() -> m_secondMechanism.setSecondMechanism(Constants.SecondMechanismConstants.SECOND_MECHANISM_PULL)),
              new ExtendClimberToHeightCommand(m_elevator, Constants.ElevatorConstants.FOURTH_RUNG_ENCODER_HEIGHT),
              new ExtendClimberToHeightCommand(m_elevator)
            )
        );

        //configure climb to third rung climb sequence 
        consoleButtons[11].whenPressed(
            new SequentialCommandGroup(
              new ExtendClimberToHeightCommand(m_elevator),
              new InstantCommand(() -> m_secondMechanism.setSecondMechanism(Constants.SecondMechanismConstants.SECOND_MECHANISM_PULL)),
              new ExtendClimberToHeightCommand(m_elevator, Constants.ElevatorConstants.THIRD_RUNG_ENCODER_HEIGHT),
              new ParallelCommandGroup(
                new ExtendClimberToHeightCommand(m_elevator),
                new InstantCommand(() -> m_secondMechanism.setSecondMechanism(Constants.SecondMechanismConstants.SECOND_MECHANISM_PUSH)))));

          //configure climb to 2 rung climb sequence 
        consoleButtons[10].whenPressed(
          new ExtendClimberToHeightCommand(m_elevator));
          //configure raise elevator before starting climb
        consoleButtons[9].whenPressed(  
          new ExtendClimberToHeightCommand(m_elevator, Constants.ElevatorConstants.SECOND_RUNG_ENCODER_HEIGHT));
        
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}

