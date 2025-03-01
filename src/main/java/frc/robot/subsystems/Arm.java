package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.ArmConstants;

import java.util.function.BooleanSupplier;


public class Arm extends SubsystemBase{

    private double armSpeed = 0;
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
    private double wristPosition; 

    //shuffleboard stuff
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private BooleanSupplier armMechAtSetpoint = ()-> false; 
    private BooleanSupplier armAtSetpoint = ()-> false; 
    private BooleanSupplier wristAtSetpoint = ()-> false; 
    private double armTargetPos = 0;
    

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
    private final ProfiledPIDController armPid = new ProfiledPIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD, new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration));
    private final PIDController wristPid = new PIDController(ArmConstants.wristkP, ArmConstants.wristkI, ArmConstants.wristkD);

/**
 * @param operator the joystick port
 * @return the action/method to run
 */
    public Command moveArm(CommandXboxController operator){
           return this.run(
            () -> {
                readFromController(operator); 
            });
    }
/**
 * Sets the target position of the arm and wrist then call set speed
 * @param operator the joystick port
 */
    private void readFromController(CommandXboxController operator){
      armTargetPos += (operator.getLeftY()/40);
        setArmMechTarget(operator.getLeftX(),armTargetPos); 
        setArmSpeed();
        setWristSpeed();
    }

  /**
   * @return runs arm motor while it is not at the setpoint
   */
  public void armPreset (){
    setArmSpeed();
  }

 /**
   * @return runs wrist motor while it is not at the setpoint
   */
  public void wristPreset (){
}
/**
 * sets arm speed based on armPid loop
 */
  private void setArmSpeed(){
      armSpeed = armPid.calculate(armPosition);
      armMotor.set(armSpeed);
  }
/**
 * sets wrist speed based on armPid loop
 */
  private void setWristSpeed(){
    wristSpeed = wristPid.calculate(wristPosition);
    wristMotor.set(wristSpeed);
}
/**
 * @return Runs speed = 0 once
 */
  public Command stopArm() {
    return this.runOnce(
        () -> {
            armSpeed = 0;
            wristSpeed = 0;
        });
  }
  public void resetPidError(){
    armPid.reset(armPosition);
}
 /**
  * sets Target position for both arm and wrist
  */
  public void setArmMechTarget(double wristTarget, double armTarget){
    setArmTarget(armTarget * 13);
    setWristTarget(wristTarget * 5);
  }
  public void setArmTarget(double target){
    armPid.setGoal(target);
  }
  public void setWristTarget(double target){
    wristPid.setSetpoint(target);
  }

  /**
   * gets encoder position and saves it as a value
   */
  private void getEncoderData(){
    //armPosition = armEncoder.get();
    armPosition = armEncoder.get();
    //wristPosition = armEncoder.get();

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
    System.out.print(LimelightHelpers.getTX("limelight"));

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

