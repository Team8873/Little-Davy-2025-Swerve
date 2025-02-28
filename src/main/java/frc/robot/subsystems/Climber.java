//aryan only
//modeled after elevator code
//don't worry about unused imports. Imported them because I thought I needed them, but there is a lot of overlap so you don't use all

package frc.robot.subsystems;

//spark max imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase; //used for getBusVoltage
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
//switched to throughbore so don't need

//shuffleboard imports but don't use for now
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//servo import
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase{ // puts climber as a subsystem; inside is code for the climber
  //introduce stuff
    private final SparkMax motorForClimber = new SparkMax (ClimberConstants.motorForClimberID, MotorType.kBrushless);//a motor motorForClimber
    private final DutyCycleEncoder encoderForClimber = new DutyCycleEncoder(ClimberConstants.encoderForClimberDIOPort); //through-bore encoder
    private final Servo climberServo = new Servo(ClimberConstants.servoID); //a servo called climberServo

    //its not at the setpoint when we turn it on
    private BooleanSupplier climberAtSetpoint = ()-> false;

    private double speed = 0;
    private double climberMotorPosition = 0;
    private double target = 0;
    private double climberVoltage = 0;
    private double servoValue = 0;
    private final BooleanSupplier somethingFalse = ()-> false; //use this because boolean supplier is weird
    //shuffleboard entries
        private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");

    private GenericEntry positionEntry =
      tab.add("Climber Position", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,1)
         .getEntry();
     private GenericEntry targetEntry =
      tab.add("Climber target", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,2)
         .getEntry();
     private GenericEntry statusEntry =
         tab.add("Servo Status", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0,3)
            .getEntry();
 
 //SERVOSTUFF
    //need to trip the servo to move the motor in positive direction. 1.0 is engaged 0.0 is disengaged
    //use 1.0 for positive direction and 0.0 for negative. It's location but this how to use it. It might be flipped around, don't know until test
    private Command engageServo(){
    climberServo.set(1.0);
    return this.runOnce(
        () -> {
            servoValue = 1.0;
        }
    );
    }
    private Command disengageServo(){
    climberServo.set(0.0);
    return this.runOnce(
        () -> {
            servoValue = 0.0;
        }
    );
    }
 
    //PID STUFF:
 private final PIDController climberPid = new PIDController(ClimberConstants.ClimberkP, ClimberConstants.ClimberkI, ClimberConstants.ClimberkD);

 //defines targetPosition as the target
 public void targetPosition(double target){
    climberPid.setSetpoint(target);} //setSetpoint sets the setpoint after you get it in getClimberSetpointStatus

 // defines set speed as making the motor go to target (speed is how much motor has to move)
 private void setSpeed(){
    speed = climberPid.calculate(climberMotorPosition);
    motorForClimber.set(speed);
    }
 public void goToTarget(){
 if(climberAtSetpoint.equals(somethingFalse)){
    targetPosition(target);
    setSpeed();
 };
}
 //get the set point and put it as the target. Return if its at the setpoint or not.
    public BooleanSupplier getClimberSetpointStatus(){
    target = climberPid.getSetpoint();
   return climberAtSetpoint = ()-> climberPid.atSetpoint();
 }

    //Y will shoot to 90 deg
    public Command moveToEngaged(){
    targetPosition(ClimberConstants.engagedPosition);
    disengageServo();
    return this.runOnce(
        ()-> {
            setSpeed();
        }
    );

    }
    //Xbutton will move down to climbed
    public Command moveClimberDown(){
        engageServo();
        return this.runOnce(
            ()-> {
                motorForClimber.set(-0.3);
            });
    }

    private void updateShuffleboardWidgetsClimber(){
        positionEntry.setDouble(climberMotorPosition);
        targetEntry.setDouble(target);
        statusEntry.setDouble(servoValue);
        
    }
//Periodically gets motorPosition, voltage, and updates shuffleboard
 @Override
    public void periodic() {
        climberMotorPosition = encoderForClimber.get();
        getClimberSetpointStatus();
        double climberVoltage = motorForClimber.getBusVoltage();
        updateShuffleboardWidgetsClimber();
}
}
