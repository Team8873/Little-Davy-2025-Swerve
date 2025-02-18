//aryan only
//modeled after elevator code

package frc.robot.subsystems;

//spark max imports
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.XboxController.Button;
//shuffleboard imports but don't use for now
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//imports constants from Constants.java
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

//Motor's resting position to be defined later (w/o power)
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//Configuration of motor when not resting(with power)
import com.revrobotics.spark.config.SparkMaxConfig;

//imports boolean supplier: used in get setpoint status. refer to line 52 and 
import java.util.function.BooleanSupplier;
//pid stuff
import edu.wpi.first.math.controller.PIDController;


public class Climber extends SubsystemBase{ // puts climber as a subsystem; inside is code for the climber
  //introduce stuff
    private final SparkMax motorForClimber = new SparkMax (ClimberConstants.motorForClimberId, MotorType.kBrushless);//a motor motorForClimber
    private RelativeEncoder encoderForClimber = motorForClimber.getEncoder(); //a relative encoder called "encoder for climber"

   //every deg is about 42/360 which is 0.1167
   //Ticks per deg converts from deg to ticks
    private final double ticksPerDegClimber = 42/360;
    //a 100:1 motor
    private final double motorRatioClimberMultiplier = 100;

   //defined positions; now everythings in ticks
    private final double restingPosition = 0*ticksPerDegClimber*motorRatioClimberMultiplier; 
    private final double engagedPosition = 90*ticksPerDegClimber*motorRatioClimberMultiplier;
    private final double climbedPositon = 45*ticksPerDegClimber*motorRatioClimberMultiplier;

    //its not at the setpoint when we turn it on
    private BooleanSupplier climberAtSetpoint = ()-> false;

    double speed = 0;
    double motorPosition = 0;
    double target = 0;
   

    //PID STUFF:
 private final PIDController climberPid = new PIDController(ClimberConstants.ClimberkP, ClimberConstants.ClimberkI, ClimberConstants.ClimberkD);

 //defines targetPosition as the target
 public void targetPosition(double target){
    climberPid.setSetpoint(target);}

 // defines set speed as making the motor go to target (speed is how much motor has to move)
 private void setSpeed(){
    speed = climberPid.calculate(motorPosition);
    motorForClimber.set(speed);
    }
 // if its at the setpoint tell it to stop
    public Command climberPreset(){
    targetPosition(restingPosition);
    return this.run(
        () -> {
            while(!climberPid.atSetpoint()){setSpeed();}
        }
    );
}
//get the set point and put it as the target. Return if its at the setpoint or not.
    public BooleanSupplier getElevatorSetpointStatus(){
    target = climberPid.getSetpoint();
   return climberAtSetpoint = ()-> climberPid.atSetpoint();
}

    //up d-pad will shoot to 90 deg
    public Command moveToEngaged(){
    targetPosition(engagedPosition);
    return this.runOnce(
        ()-> {
            setSpeed();
        }
    );

    }
    //Xbutton will shoot down to climbed
    public Command moveToClimbed(){
        targetPosition(climbedPositon);
        return this.runOnce(
            ()-> {
                setSpeed();
            }
        );
    }
    //down d-pad will move down
    public Command moveClimberDown(){
        return this.runOnce(
            ()-> {
                motorForClimber.set(-0.2);
            });
    }
//Periodically gets motorPosition
 @Override
    public void periodic() {
        motorPosition = encoderForClimber.getPosition();
        getElevatorSetpointStatus();
}
}
