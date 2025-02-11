package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import static frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase{
    private double armSpeed = 0;
    private double wristSpeed = 0;
    private final SparkMax armMotor = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless); //create the motor object
    private final SparkMax wristMotor = new SparkMax(ArmConstants.wristCanId, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private double armPosition;
    private double wristPosition; 
    

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
        armSpeed = operator.getRightY();
        wristSpeed = operator.getRightX();
        setSpeed();
    }

    private void setSpeed(){
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
  
  private void getEncoderData(){
    armPosition = armEncoder.getPosition();
    wristPosition = wristEncoder.getPosition();

  }
  public void createWidget(){
    Shuffleboard.getTab("Subsystems").add("Arm",armSpeed).withWidget(BuiltInWidgets.kNumberBar).withPosition(1, 3);
    Shuffleboard.getTab("Subsystems").add("Wrist",wristSpeed).withWidget(BuiltInWidgets.kNumberBar).withPosition(1, 4);
  }
  @Override
  public void periodic (){
    getEncoderData();
  } 
}

