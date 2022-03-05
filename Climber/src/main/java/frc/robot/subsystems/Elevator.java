

// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


// import frc.robot.commands.*;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commands.ExtendClimberToMidRungCommand;
import frc.commands.ReachToNextRungCommand;
import frc.commands.RetractClimberFullCommand;
import frc.commands.RetractClimberMinimumCommand;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.networktables.NetworkTableEntry;
import static frc.robot.Constants.TUNING;
import static frc.robot.Constants.SLOT_INDEX;
import static frc.robot.Constants.ElevatorConstants.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.util.Map;


/**
 *
 */
public class Elevator extends SubsystemBase {
    private NetworkTableEntry elevatorMotorPowerNT;
    private NetworkTableEntry positionSetPointNT;
    private NetworkTableEntry FConstantNT;
    private NetworkTableEntry PConstantNT;
    private NetworkTableEntry IConstantNT;
    private NetworkTableEntry DConstantNT;
    private WPI_TalonFX leftElevatorMotor;
    private WPI_TalonFX rightElevatorMotor;
    private final Pigeon2 m_pigeon = new Pigeon2(PIGEON_ID);
    private double encoderPositionSetpoint;


public Elevator() {

    this.leftElevatorMotor = new WPI_TalonFX(LEFT_ELEVATOR_MOTOR_CAN_ID); 
    this.rightElevatorMotor = new WPI_TalonFX(RIGHT_ELEVATOR_MOTOR_CAN_ID); 
    

        //set directions
        this.leftElevatorMotor.setInverted(true);
        /* Factory Default all hardware to prevent unexpected behaviour */
        this.rightElevatorMotor.configFactoryDefault();
        this.leftElevatorMotor.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        this.rightElevatorMotor.configNeutralDeadband(0.001);
        this.leftElevatorMotor.configNeutralDeadband(0.001);
        
        /* Config sensor used for Primary PID [Velocity] */
        this.rightElevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                SLOT_INDEX,
                kTimeoutMs);
        this.leftElevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,
                SLOT_INDEX, kTimeoutMs);
    


        /* Config the peak and nominal outputs */
	    this.rightElevatorMotor.configNominalOutputForward(0, kTimeoutMs);
	    this.rightElevatorMotor.configNominalOutputReverse(0, kTimeoutMs);
		this.rightElevatorMotor.configPeakOutputForward(1, kTimeoutMs);
		this.rightElevatorMotor.configPeakOutputReverse(-1, kTimeoutMs);
        this.leftElevatorMotor.configNominalOutputForward(0, kTimeoutMs);
		this.leftElevatorMotor.configNominalOutputReverse(0, kTimeoutMs);
		this.leftElevatorMotor.configPeakOutputForward(1, kTimeoutMs);
		this.leftElevatorMotor.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		this.rightElevatorMotor.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
		this.rightElevatorMotor.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
		this.rightElevatorMotor.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
		this.rightElevatorMotor.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);
        this.leftElevatorMotor.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
		this.leftElevatorMotor.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
		this.leftElevatorMotor.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
		this.leftElevatorMotor.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        
         /*configure the magic motion profile*/
         this.leftElevatorMotor.configMotionSCurveStrength(SCURVE_STRENGTH, kTimeoutMs);
         this.rightElevatorMotor.configMotionSCurveStrength(SCURVE_STRENGTH, kTimeoutMs);
         this.leftElevatorMotor.configMotionCruiseVelocity(MAX_ELEVATOR_VELOCITY, kTimeoutMs);
         this.rightElevatorMotor.configMotionCruiseVelocity(MAX_ELEVATOR_VELOCITY, kTimeoutMs);
         this.leftElevatorMotor.configMotionAcceleration(ELEVATOR_ACCELERATION, kTimeoutMs);
         this.rightElevatorMotor.configMotionAcceleration(ELEVATOR_ACCELERATION, kTimeoutMs);

        /* zero the sensors*/
        leftElevatorMotor.setSelectedSensorPosition(0, kPIDLoopIdx,  kTimeoutMs);

         
        Shuffleboard.getTab("Elevator").addNumber("Encoder Value", this :: getElevatorEncoderHeight);
        Shuffleboard.getTab("Elevator").addNumber("Pitch Value", m_pigeon :: getPitch);
        Shuffleboard.getTab("Elevator").add("Extend Climber to Mid", new ExtendClimberToMidRungCommand(this));
        Shuffleboard.getTab("Elevator").add("Reach to Next Rung", new ReachToNextRungCommand(this));
        Shuffleboard.getTab("Elevator").add("Retract Climber Full", new RetractClimberFullCommand(this));
        Shuffleboard.getTab("Elevator").add("Retract Climber Minimum", new RetractClimberMinimumCommand(this));

        this.elevatorMotorPowerNT = Shuffleboard.getTab("Elevator")
            .add("Elevator Motors", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)) //FIX_ME figure max motor power should be 1
            .getEntry();

            
        this.positionSetPointNT = Shuffleboard.getTab("Elevator")
            .add("Position Setpoint", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", MAX_ELEVATOR_HEIGHT))
            .getEntry();

        this.FConstantNT = Shuffleboard.getTab("Elevator")
                .add("Flywheel F", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.PConstantNT = Shuffleboard.getTab("Elevator")
                .add("Flywheel P", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.IConstantNT = Shuffleboard.getTab("Elevator")
                .add("Flywheel I", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();

        this.DConstantNT = Shuffleboard.getTab("Elevator")
                .add("Flywheel D", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
                .getEntry();


        this.encoderPositionSetpoint = 0.0;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (TUNING) {
         // when tuning, we first set motor power and check the resulting velocity
         // once we have determined our feedforward constant, comment the following lines
         // and uncomment the ones to tune the PID
        double motorPower = this.elevatorMotorPowerNT.getDouble(0.0); 
        this.leftElevatorMotor.set(ControlMode.PercentOutput, motorPower);
        this.rightElevatorMotor.set(ControlMode.PercentOutput, motorPower);


            // this.rightElevatorMotor.config_kF(SLOT_INDEX, this.FConstantNT.getDouble(0.0), kTimeoutMs);
            // this.rightElevatorMotor.config_kP(SLOT_INDEX, this.PConstantNT.getDouble(0.0), kTimeoutMs);
            // this.rightElevatorMotor.config_kI(SLOT_INDEX, this.IConstantNT.getDouble(0.0), kTimeoutMs);
            // this.rightElevatorMotor.config_kD(SLOT_INDEX, this.DConstantNT.getDouble(0.0), kTimeoutMs);
            // this.leftElevatorMotor.config_kF(SLOT_INDEX, this.FConstantNT.getDouble(0.0), kTimeoutMs);
            // this.leftElevatorMotor.config_kP(SLOT_INDEX, this.PConstantNT.getDouble(0.0), kTimeoutMs);
            // this.leftElevatorMotor.config_kI(SLOT_INDEX, this.IConstantNT.getDouble(0.0), kTimeoutMs);
            // this.leftElevatorMotor.config_kD(SLOT_INDEX, this.DConstantNT.getDouble(0.0), kTimeoutMs);

            // double desiredEncoderPosition = this.positionSetPointNT.getDouble(0.0);
            // this.setElevatorMotorPosition(desiredEncoderPosition);


        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

   

    public double getElevatorEncoderHeight(){
        return this.rightElevatorMotor.getSelectedSensorPosition();
    }

    public void setElevatorMotorPower(double power){
        this.leftElevatorMotor.set(ControlMode.PercentOutput, power);
        this.rightElevatorMotor.set(ControlMode.PercentOutput, power);
    }

    public void setElevatorMotorPosition(double desiredEncoderPosition) {
        this.encoderPositionSetpoint = desiredEncoderPosition;
        this.leftElevatorMotor.set(ControlMode.MotionMagic, desiredEncoderPosition);
        this.rightElevatorMotor.set(ControlMode.MotionMagic, desiredEncoderPosition);
    }

    public boolean atSetpoint(){
        return Math.abs(this.getElevatorEncoderHeight() - this.encoderPositionSetpoint) < ELEVATOR_POSITION_TOLERANCE;
    }

    public void disableElevator() {
        this.leftElevatorMotor.set(ControlMode.PercentOutput, 0.0);
        this.rightElevatorMotor.set(ControlMode.PercentOutput,0.0);
    }

    public boolean atPitch(){
        return Math.abs(m_pigeon.getPitch() - PITCH_SETPOINT) < PITCH_TOLERANCE;
    }
}



