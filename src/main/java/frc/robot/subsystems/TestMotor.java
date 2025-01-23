package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TestMotor extends SubsystemBase{
    private double speed = 0;
    final double speedMult = .2;
    private final SparkMax motor = new SparkMax(15, MotorType.kBrushless);
//   public Command runForward() {
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return this.runOnce(
//         () -> {
//             motor.set(.5);
//         });
//   }
//   public Command runBackwards() {
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return this.run(
//         () -> {
//             motor.set(-.5);
//         });
//   }
/**
 * @param drive the joystick port
 * @return the action to run 
 */
  public Command driveMotor(CommandXboxController drive){
           return this.run(
            () -> {
                readFromController(drive); 
            });
      }
      public void setSpeed(){
                 motor.set(speed);
             }
/**
 * Controls the acceleration of Neo by adding and subtracting the trigger axis
 * @param drive controller port
 */
  public void readFromController(CommandXboxController drive){
        speed -= (drive.getLeftTriggerAxis()/100);
        speed += (drive.getRightTriggerAxis()/100);
        setSpeed();
    }

/**
 * @return Runs speed = 0 once
 */
  public Command stopMotor() {
    return this.runOnce(
        () -> {
            speed = 0;
        });
  }
 

}
