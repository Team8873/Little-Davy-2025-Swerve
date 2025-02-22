//aryan only
//modeled after elevator code
//don't worry about unused imports. Imported them because I thought I needed them, but there is a lot of overlap so you don't use all

package frc.robot.subsystems;

//spark max imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import com.revrobotics.RelativeEncoder;
//relative encoder is magic import. has pretty much all encoder stuff but I had other stuff imported before I saw this one. Need this one because has RelativeEncoder


//shuffleboard imports but don't use for now
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//controller import: dont need because controller stuff is in robot container but leave for now just in case
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

//imports constants from Constants.java
import frc.robot.Constants.ClimberConstants;



//imports boolean supplier: used in get setpoint status
import java.util.function.BooleanSupplier;
//pid import
import edu.wpi.first.math.controller.PIDController;

//servo import
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase{ // puts climber as a subsystem; inside is code for the climber
  //introduce stuff
    private final SparkMax motorForClimber = new SparkMax (ClimberConstants.motorForClimberID, MotorType.kBrushless);//a motor motorForClimber
    private RelativeEncoder encoderForClimber = motorForClimber.getEncoder(); //a relative encoder called "encoder for climber"
    private Servo climberServo = new Servo(ClimberConstants.servoID); //a servo called climberServo

    //its not at the setpoint when we turn it on
    private BooleanSupplier climberAtSetpoint = ()-> false;

    private double speed = 0;
    private double motorPosition = 0;
    private double target = 0;
 
 //SERVOSTUFF
    //need to trip the servo to move the motor in positive direction. 1.0 is engaged 0.0 is disengaged
    //use 1.0 for positive direction and 0.0 for negative. It's location but this how to use it. It might be flipped around, don't know until test
    private void engageServo(){
    climberServo.set(1.0);
    }
    private void disengageServo(){
    climberServo.set(0.0);
    }
 

    //PID STUFF:
 private final PIDController climberPid = new PIDController(ClimberConstants.ClimberkP, ClimberConstants.ClimberkI, ClimberConstants.ClimberkD);

 //defines targetPosition as the target
 public void targetPosition(double target){
    climberPid.setSetpoint(target);} //setSetpoint sets the setpoint after you get it in getClimberSetpointStatus

 // defines set speed as making the motor go to target (speed is how much motor has to move)
 private void setSpeed(){
    speed = climberPid.calculate(motorPosition);
    motorForClimber.set(speed);
    }
 // while it's not at the setpoint set the speed to get to the setpoint
 //will set to 0 if no setpoint because in beginning speed = 0
    public Command climberPreset(){
    targetPosition(ClimberConstants.restingPosition);
    return this.run(
        () -> {
            while(!climberPid.atSetpoint()){setSpeed();}
        }
    );
}
//get the set point and put it as the target. Return if its at the setpoint or not.
    public BooleanSupplier getClimberSetpointStatus(){
    target = climberPid.getSetpoint();
   return climberAtSetpoint = ()-> climberPid.atSetpoint();
}

    //up d-pad will shoot to 90 deg
    public Command moveToEngaged(){
    targetPosition(ClimberConstants.engagedPosition);
    disengageServo();
    return this.runOnce(
        ()-> {
            setSpeed();
        }
    );

    }
    //Xbutton will shoot down to climbed
    public Command moveToClimbed(){
        targetPosition(ClimberConstants.climbedPositon);
        engageServo();
        return this.runOnce(
            ()-> {
                setSpeed();
            }
        );
    }
    //down d-pad will move down
    public Command moveClimberDown(){
        disengageServo();
        return this.runOnce(
            ()-> {
                motorForClimber.set(-0.2);
            });
    }
//Periodically gets motorPosition
 @Override
    public void periodic() {
        motorPosition = encoderForClimber.getPosition();
        getClimberSetpointStatus();
}
}
