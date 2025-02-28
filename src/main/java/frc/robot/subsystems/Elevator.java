package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class Elevator extends SubsystemBase{

    // creates Sparkmax object 
    private final SparkMax motorLeft = new SparkMax(ElevatorConstants.elevatorLCanId, MotorType.kBrushless);
    private final SparkMax leadMotorRight = new SparkMax(ElevatorConstants.elevatorRCanId, MotorType.kBrushless);
    // creates private variables
    private BooleanSupplier elevatorAtSetpoint = ()-> false;
    private boolean configured = false;
    private double speed = 0; 
    private double elevatorRightPos= 0;
    private double elevatorLeftPos = 0;
    private double elevatorRightVel = 0;
    private double elevatorLeftVel = 0;
    private double elevatorVelocity = 0;

    // shuffleboard thing I don't know what shuffleboard is
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");

    private GenericEntry speedEntry =
      tab.add("Elevator Speed", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,1)
         .getEntry();

    private GenericEntry positionEntry =
      tab.add("Elevator position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(2,0)
         .getEntry();

    private GenericEntry targetEntry =
      tab.add("Elevator busVolt", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(2,1)
         .getEntry();

    private GenericEntry elevatorSetpointWidget =
      tab.add("Elevator atSetpoint", false)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(3,5)
         .getEntry();
         
    private GenericEntry elevatorPast =
      tab.add("Elevator Past", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(4,5)
         .getEntry();
    

        
    //gets the encoders and creates a object to talk to the encoders
    //private RelativeEncoder elevatorEncoder = motorLeft.getEncoder();
    private RelativeEncoder elevatorEncoder = leadMotorRight.getEncoder();
    private RelativeEncoder elevatorLeftEncoder = motorLeft.getEncoder();

    //creates PID loop
    private final ProfiledPIDController elevatorPid = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
    new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity,ElevatorConstants.maxAcceleration));
    //private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    private ComplexWidget pidEntry =
      tab.add("Elevator Pid", elevatorPid)
        .withWidget(BuiltInWidgets.kPIDController)
         .withPosition(6,1);
         
    //defines variables to 0 
    private double elevatorPosition = 0;
    private State targetPosition;
    private double pastPosition = 0;
    private boolean brakeOn = false;
    public Elevator(){
        //elevatorPid.enableContinuousInput(0, ElevatorConstants.maxElevatorInput);
    }

    //sets follower
    public void setFollower(){
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig FollowerConfig = new SparkMaxConfig();
    
        /*
         * Set parameters that will apply to all SPARKs. We will also use this as
         * the left leader config.
         */
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
    
        // Apply the global config and invert since it is on the opposite side
        FollowerConfig
            .apply(globalConfig)
            .inverted(true)
            ;

             /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
        leadMotorRight.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorLeft.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        configured = true;
        
    }

   //if not configured set to follower
    public void checkIfSetFollow(){
        if(!configured){
            setFollower();
        }
    }
    
    /**
     * @param operator the joystick to read from
     * @return the action/method to run
     */
    public Command moveElevator(CommandXboxController operator){
        return this.run(
            () -> {
            readFromController(operator);
            });
    }
    // private void stopElevatorFall(CommandXboxController operator){
    //     while((operator.getRightY() < 0.1 || operator.getRightY() > -0.1) && pastPosition > 1)
    //     {
    //         speed = elevatorPid.calculate(elevatorPosition);
    //         brakeOn = true;
    //     }
    // }
    

    /**
     * Calls targetposition and sets the target to be the y values of the left joystick 
     * sets elevator motor speed
     * @param operator the joystick to read from
     */
    private void readFromController(CommandXboxController operator){
        targetElevatorPosition((operator.getRightY() + 2.0) * 2.5);
        //stopElevatorFall(operator);
        //targetPosition(pastPosition);
        setSpeed();
    }

    /**
     * @return while the elevator pid is NOT at the setpoint it runs the motor
     */
    public void elevatorPreset (){
        goToPreset();
    }

    /**
     * Sets speed equal to Pid Calculation
     * sets motor to referenced speed
     */
    private void setSpeed(){
        leadMotorRight.set(speed);
        motorLeft.set(speed);
    }
    private void goToPreset(){
        speed = elevatorPid.calculate(elevatorPosition);
        //+ m_feedforward.calculate(elevatorPid.getSetpoint().velocity);
        setSpeed();
    }
    /**
     * sets elevator position equal to encoder position
     */
    private void getEncoderData(){
        elevatorRightPos = elevatorEncoder.getPosition();
        elevatorLeftPos = elevatorLeftEncoder.getPosition();
        elevatorPosition = (elevatorLeftPos + elevatorRightPos) / 2;

        elevatorRightVel = elevatorEncoder.getVelocity();
        elevatorLeftVel = elevatorLeftEncoder.getVelocity();

        elevatorVelocity = (elevatorLeftVel + elevatorRightVel)/2;

        elevatorPosition *= ElevatorConstants.gearRatio;
        
    }
    
    /**
     * sets the setpoint for the Elevator PID controller
     * @param target the target position
     */
    public void targetElevatorPosition(double target){
        //if(target < 0){target = 0.3;}
        elevatorPid.setGoal(target);
    }
    public void resetPidError(){
        elevatorPid.reset(elevatorPosition, elevatorVelocity);
    }
    /**
     * setting the targetPosition to the current set point 
     * @return the boolean value if whether the elevator is at the set point or not
     */
    public BooleanSupplier getElevatorSetpointStatus(){
        targetPosition = elevatorPid.getGoal();
       return elevatorAtSetpoint = ()-> elevatorPid.atSetpoint();
    }

    //creates an unchangeable trigger boolean if whether the elevator is at the setpoint
    public final Trigger elevatorAtTarget = new Trigger(getElevatorSetpointStatus());

    @Override
  public void periodic(){
    getEncoderData();
    getElevatorSetpointStatus();
    updateShuffleboardWidgets();
  } 

  /**
   * updates the shuffleboard stuff
   */
  private void updateShuffleboardWidgets(){
    positionEntry.setDouble(elevatorPosition);
    speedEntry.setDouble(speed);
    targetEntry.setDouble(leadMotorRight.getBusVoltage());
    elevatorSetpointWidget.setBoolean(elevatorAtSetpoint.getAsBoolean());
    elevatorPast.setDouble(pastPosition);


  }

}
