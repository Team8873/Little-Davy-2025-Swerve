package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TestMotor;
public class testMotorCommandInput extends Command{

    //private double speed;

    public TestMotor motor;
    /**
     * creates and tells where the motor is reading from on CAN
     * @param drive the joystick
     * @return the trigger axis
     */
    public testMotorCommandInput readFromController(CommandXboxController drive){
         motor = new TestMotor();
         motor.setSpeed(drive.getRightTriggerAxis());
        
        return new testMotorCommandInput(); 
        
    }

}
