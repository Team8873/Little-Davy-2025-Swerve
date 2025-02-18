package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private double speed = 0;
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless); //create the motor object
    private final SparkMax humanMotor = new SparkMax(IntakeConstants.humanIntakeCanId, MotorType.kBrushless);
    private final PIDController humanPid = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private final RelativeEncoder humanEncoder = humanMotor.getEncoder();
    private double humanEncoderPosition = 0;
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private GenericEntry speedEntry =
      tab.add("Intake Motor", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,2)
         .getEntry();
    private GenericEntry humanEntry =
      tab.add("Human Intake Up", 0)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(0,3)
         .getEntry();
/**
 * @return the action to run 
 */
  public Command runIntake(){
    speed = 1;
      return this.run(
          () -> {
            setSpeed();
          });
  }

/**
 * @return Runs speed = 0 once
 */
  public Command stopIntake() {
    speed = 0;
    return this.runOnce(
        () -> {
            
            setSpeed();
        });
  }
  
  public Command intakeEject(){
    speed = -1;
    return this.runOnce(
        () -> { 
            
            setSpeed();
        });
  }

  private void setSpeed(){
    intakeMotor.set(speed);
  }
  private void getHumanPosition(){
    humanEncoderPosition = humanEncoder.getPosition();
  }
  public void humanTargetPosition(double target){
    humanPid.setSetpoint(target);
  }
public Command moveHumanMotor(){
  return this.run(
    () -> { 
        humanMotor.set(humanPid.calculate(humanEncoderPosition));
    });
     
}
  @Override
  public void periodic(){
    speedEntry.setDouble(speed);
    humanEntry.setBoolean(humanPid.atSetpoint());
    getHumanPosition();
  } 
 
}

