package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private double speed = 0;
    private final SparkMax motor = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless); //create the motor object

/**
 * @return the action to run 
 */
  public Command runIntake(){
      return this.run(
          () -> {
            speed = 1;
            setSpeed();
          });
  }

/**
 * @return Runs speed = 0 once
 */
  public Command stopIntake() {
    return this.runOnce(
        () -> {
            speed = 0;
            setSpeed();
        });
  }
  public Command intakeEject(){
    return this.runOnce(
        () -> { 
            speed = -1;
            setSpeed();
        });
  }
  private void setSpeed(){
    motor.set(speed);
  }
  
  @Override
  public void periodic(){

  } 
 
}

