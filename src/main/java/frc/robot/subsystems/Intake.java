package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private double speed = 0;
    private final SparkMax motor = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless); //create the motor object
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private GenericEntry speedEntry =
      tab.add("Intake Motor", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,2)
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
    motor.set(speed);
  }

  @Override
  public void periodic(){
    speedEntry.setDouble(speed);
  } 
 
}

