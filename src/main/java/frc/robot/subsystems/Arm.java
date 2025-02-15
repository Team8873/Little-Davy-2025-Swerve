package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

import static frc.robot.Constants.ArmConstants;

import java.util.function.BooleanSupplier;


public class Arm extends SubsystemBase{

    private double armSpeed = 0;
    private double wristSpeed = 0;

    private final SparkMax armMotor = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless); //create the motor object
    private final SparkMax wristMotor = new SparkMax(ArmConstants.wristCanId, MotorType.kBrushless);

    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

    private double armPosition;
    private double wristPosition; 

    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private BooleanSupplier armMechAtSetpoint = ()-> false; 


    private GenericEntry armPosWidget =
      tab.add("arm position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,3)
         .getEntry();
         
    private GenericEntry wristPosWidget =
      tab.add("wrist position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,4)
         .getEntry();

    private final PIDController armPid = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);
    private final PIDController wristPid = new PIDController(ArmConstants.wristkP, ArmConstants.wristkI, ArmConstants.wristkD);

/**
 * @param drive the joystick port
 * @return the action to run 
 */
    public Command moveArm(CommandXboxController drive){
           return this.run(
            () -> {
                readFromController(drive); 
            });
    }
/**
 * Controls the acceleration of Neo by adding and subtracting the trigger axis
 * @param drive controller port
 */
    private void readFromController(CommandXboxController operator){
        setArmTarget(operator.getRightX(),operator.getRightY());
        
        setSpeed();
    }
    public Command elevatorPreset (){
      return this.run(
          () -> {
              while(!armPid.atSetpoint()&!wristPid.atSetpoint()){setSpeed();}
          }
      );
  }

    private void setSpeed(){
        armSpeed = armPid.calculate(armPosition);
        wristSpeed = wristPid.calculate(wristPosition);
        armMotor.set(armSpeed);
        wristMotor.set(wristSpeed);
    }

/**
 * @return Runs speed = 0 once
 */
  public Command stopArm() {
    return this.runOnce(
        () -> {
            armSpeed = 0;
        });
  }
  public void setArmTarget(double wristTarget, double armTarget){
    armPid.setSetpoint(armTarget);
    wristPid.setSetpoint(wristTarget);
  }

  private void getEncoderData(){
    armPosition = armEncoder.getPosition();
    wristPosition = wristEncoder.getPosition();
  }
  public BooleanSupplier getArmMechSetpointStatus(){
        return armMechAtSetpoint = ()-> armPid.atSetpoint() & wristPid.atSetpoint();
    }
  public final Trigger armAtTarget = new Trigger(getArmMechSetpointStatus());

  @Override
  public void periodic (){
    getEncoderData();
    armPosWidget.setDouble(armPosition);
    wristPosWidget.setDouble(wristPosition);
    getArmMechSetpointStatus();

  } 

}

