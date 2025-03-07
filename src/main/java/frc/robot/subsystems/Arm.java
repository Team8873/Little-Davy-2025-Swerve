package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ArmConstants;

import java.util.function.BooleanSupplier;


public class Arm extends SubsystemBase{

    private double wristSpeed = 0;

    //creates the Motor objects using sparkMax
    private final SparkMax armMotor = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless); //create the motor object
    private final SparkMax wristMotor = new SparkMax(ArmConstants.wristCanId, MotorType.kBrushless);

    //get the encoders plugged into the sparkMax or connected to it
    //private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.encoderId);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    //private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);



    private double armPosition;
    private double armPreviousPosition;
    private double armVelocity;


    private double wristPosition; 

    //shuffleboard stuff
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private BooleanSupplier armMechAtSetpoint = ()-> false; 
    private BooleanSupplier armAtSetpoint = ()-> false; 
    private BooleanSupplier wristAtSetpoint = ()-> false; 
    private double armTargetPos = 0.35;
    private double wristTargetPos = 0.35;

    

    private GenericEntry armPosWidget =
      tab.add("arm position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(1,3)
         .getEntry();
         
    private GenericEntry wristPosWidget =
      tab.add("wrist position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,4)
         .getEntry();
    private GenericEntry armSetpointWidget =
      tab.add("arm atSetpoint", false)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(3,3)
         .getEntry();
         
    private GenericEntry wristSetpointWidget =
      tab.add("wrist atSetpoint", false)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(3,4)
         .getEntry();
         private GenericEntry armTargetWidget =
         tab.add("arm Target", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4,4)
            .getEntry();
            
    //creates the PID loop for the arm and wrist motors

    // private final ProfiledPIDController armPid = new ProfiledPIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
    private final PIDController armPid = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);
    private final PIDController wristPid = new PIDController(ArmConstants.wristkP, ArmConstants.wristkI, ArmConstants.wristkD);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
  
    private ComplexWidget pidEntry =
          tab.add("arm Pid", armPid)
            .withWidget(BuiltInWidgets.kPIDController)
            .withPosition(8, 1);
            private ComplexWidget wpidEntry =
          tab.add("wrist Pid", wristPid)
            .withWidget(BuiltInWidgets.kPIDController)
            .withPosition(8, 1);
    public Arm(){
      armPid.setTolerance(.005);
      armPid.disableContinuousInput();
      armPid.setSetpoint(armTargetPos);
      tab.addDouble("Arm Target Real", () -> armPid.getSetpoint());

    }
/**
 * @param operator the joystick port
 * @return the action/method to run
 */
    public Command moveArm(CommandXboxController operator){
           return this.run(
            () -> {
                  // armMotor.set(operator.getLeftY() / 5.0);
                  // return;
                readFromController(operator); 
            });
    }
    public Command kcikArm(){
      return this.run(
       () -> {
             setArmTarget(.29);
             setArmSpeed();
       });
}
/**
 * Sets the target position of the arm and wrist then call set speed
 * @param operator the joystick port
 */
    private void readFromController(CommandXboxController operator){
      armTargetPos += (-operator.getLeftY()/120);
      wristTargetPos += (operator.getLeftX()/80);
        setArmMechTarget(wristTargetPos,armTargetPos); 
        setArmSpeed();
        setWristSpeed();
    }


/**
 * sets arm speed based on armPid loop
 */
  public void setArmSpeed(){
      double armSpeed = armPid.calculate(armPosition) + m_feedforward.calculate(armPosition, armVelocity);
      armMotor.set(armSpeed);
  }
/**
 * sets wrist speed based on armPid loop
 */
  public void setWristSpeed(){
    wristSpeed = wristPid.calculate(wristPosition);
    wristMotor.set(wristSpeed);
}

  public void resetPidError(){
    armPid.reset();
}


 /**
  * sets Target position for both arm and wrist
  */
  public void setArmMechTarget(double wristTarget, double armTarget){
    setArmTarget(armTarget);
    setWristTarget(wristTarget);
  }
  public void setArmTarget(double target){
    double m_target = target;
        if(m_target < 0.1){m_target = 0.1;}
    armPid.setSetpoint(target);
    armTargetPos = m_target;
  }
  public void setWristTarget(double target){
    double m_target = target;
        if(m_target < -8){m_target = -8;}
        if(m_target > 4.22){m_target = 4.22;}
    wristPid.setSetpoint(target);
    wristTargetPos = m_target;
  }
  public double getArmPos(){
    return armPosition;
  }

  public double getArmTarget() {
    return armPid.getSetpoint();
  }
  public double getwristPos(){
    return wristPosition;
  }

  /**
   * gets encoder position and saves it as a value
   */
  private void getEncoderData(){
    //armPosition = armEncoder.get();
    armPosition = armEncoder.get();
    //wristPosition = armEncoder.get();
    armVelocity = armPosition - armPreviousPosition;
    armPreviousPosition = armPosition;
    wristPosition = wristEncoder.getPosition();
  }

  /**
   * gets arm and wrist position status
   * @return returns status of the whole arm Mech
   */
  public BooleanSupplier getArmMechSetpointStatus(){
    getArmSetpointStatus();
    getWristSetpointStatus();
        return armMechAtSetpoint = ()-> armAtSetpoint.getAsBoolean() && wristAtSetpoint.getAsBoolean();
    }
/**
 * @return returns the arm position status as a boolean
 */
  public BooleanSupplier getArmSetpointStatus(){
    return armAtSetpoint = ()-> armPid.atSetpoint();
  }
/**
 * @return returns wrist position status as a boolean
 */
  public BooleanSupplier getWristSetpointStatus(){
    return wristAtSetpoint = ()-> wristPid.atSetpoint();
  }
  public double getWristPosition(){
    return wristPid.getSetpoint();
  }
  //creates a trigger for armMech Status that can be used to automatic run command when true
  public final Trigger armMechAtTarget = new Trigger(getArmMechSetpointStatus());

  @Override
  public void periodic (){
    getEncoderData();
    updateShuffleboardWidgets();
    getArmMechSetpointStatus();
  } 
  /**
   * updates shuffleboard widgets
   */
  private void updateShuffleboardWidgets(){
    armPosWidget.setDouble(armPosition);
    wristPosWidget.setDouble(wristPosition);
    armSetpointWidget.setBoolean(armAtSetpoint.getAsBoolean());
    wristSetpointWidget.setBoolean(wristAtSetpoint.getAsBoolean());
  }

}

