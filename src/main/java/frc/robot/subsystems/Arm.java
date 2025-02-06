package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private double speed = 0;
    private final SparkMax motor = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless); //create the motor object

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
    private void readFromController(CommandXboxController drive){
        speed = drive.getRightY();
        setSpeed();
    }

    private void setSpeed(){
                 motor.set(speed);
    }

/**
 * @return Runs speed = 0 once
 */
  public Command stopArm() {
    return this.runOnce(
        () -> {
            speed = 0;
        });
  }
  public void createWidget(){
    Shuffleboard.getTab("Subsystems").add("Arm",motor).withWidget(BuiltInWidgets.kNumberBar);
  }
}

