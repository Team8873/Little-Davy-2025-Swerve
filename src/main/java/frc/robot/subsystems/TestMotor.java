package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class TestMotor extends SubsystemBase{
    private double speed = 0;
    final double speedMult = .2;
    private final SparkMax motor = new SparkMax(15, MotorType.kBrushless);
  /**
   * Example command factory method.
   *
   * @return a command
   */
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
  public Command driveMotor(CommandXboxController drive){
           return this.run(
            () -> {
                readFromController(drive); 
            });
      }
      public void setSpeed(){
                 motor.set(speed);
             }
  public void readFromController(CommandXboxController drive){
        speed -= (drive.getLeftTriggerAxis()/100);
        speed += (drive.getRightTriggerAxis()/100);
        setSpeed();
    }

    
  public Command stopMotor() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.run(
        () -> {
            speed = 0;
        });
  }
 

}
//     private SparkMax motor;

//     //creates motor
//     public TestMotor(){
//         motor = new SparkMax(14, MotorType.kBrushless);
        
//     }
    

//     
//     public Command readFromController(CommandXboxController drive){
        
//         setSpeed(drive.getRightTriggerAxis());
       
       
       
//    }

    
// }
