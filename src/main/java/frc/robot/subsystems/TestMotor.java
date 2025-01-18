package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.testMotorCommandInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class TestMotor extends SubsystemBase{
    
    private SparkMax motor;

    //creates motor
    public TestMotor(){
        motor = new SparkMax(14, MotorType.kBrushless);
        
    }
    

    public void setSpeed(Double speed){
        motor.set(speed);
        
    }

    
}
