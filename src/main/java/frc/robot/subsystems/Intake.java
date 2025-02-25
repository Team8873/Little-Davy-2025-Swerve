package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private double speed = 0;

    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless); //create the motor object
    private final SparkMax humanMotor = new SparkMax(IntakeConstants.humanIntakeCanId, MotorType.kBrushless);

    // creates PIDController object
    private final PIDController humanPid = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    // create RelativeEncoder object 
    private final RelativeEncoder humanEncoder = humanMotor.getEncoder();

    //creates variables
    private double humanEncoderPosition = 0;
    private BooleanSupplier humanAtSetpoint = ()-> false;
    // creates shuffleboard stuff
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private GenericEntry speedEntry =
      tab.add("Intake Motor", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,2)
         .getEntry();
    private GenericEntry humanEntry =
      tab.add("Human Intake Up", 0)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(0,3)
         .getEntry();
         private GenericEntry currentWid =
      tab.add("Intake Speed", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(0,2)
         .getEntry();
         
/**
 * sets speed to 1
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
 * sets speed to 0
 * @return the action to run 
 */
  public Command stopIntake() {
    speed = 0;
    return this.runOnce(
        () -> {
            setSpeed();
        });
  }
  /**
   * sets speed 0.15
   * @return references lastest object(intake) and runs the motor
   */
  public Command holdIntake(){
    speed = 0.15;
    return this.run(
      () -> {
        setSpeed();
      }
    );
  }

  /**
   * sets speed to -1
   * @return references latest object(intake) and runs the motor
   */
  public Command intakeEject(){
    speed = -1;
    return this.runOnce(
        () -> { 
            setSpeed();
        });
  }
  public Command moveIntake(CommandXboxController operator){
    return this.run(
      ()-> {
        speed = operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
        setSpeed();
      }
    );
  }
  
  
  /**
   * sets the speed of the intake motor
   */
  private void setSpeed(){
    intakeMotor.set(speed);
  }
/**
 * get the position of the human encoder and sets it equal the variable humanEncoderPosition
 */
  private void getHumanPosition(){
    humanEncoderPosition = humanEncoder.getPosition();
  }
/**
 * sets the target position of the humanPid
 * @param target the target position
 */
  public void humanTargetPosition(double target){
      humanPid.setSetpoint(target);
  }
/**
 * @return references the latest object(intake) then runs the humanMotor based of the calculation of the Pid Loop
 */
  public Command moveHumanMotor(){
    return this.run(
      () -> { 
          humanMotor.set(humanPid.calculate(humanEncoderPosition));
      });
  }
/**
 * equates the variable humanAtSetpoint to the boolean value of whether the position is within the error parameters
 */
  private void humanAtSetpointStatus(){
    humanAtSetpoint = ()-> humanPid.atSetpoint();
  }
  //creates trigger object
  public final Trigger humanIntakeAtPos = new Trigger(humanAtSetpoint);

// things that get called every 20 ms
  @Override
  public void periodic(){
    speedEntry.setDouble(speed);
    humanEntry.setBoolean(humanAtSetpoint.getAsBoolean());
    currentWid.setDouble(intakeMotor.getAppliedOutput());
    getHumanPosition();
    humanAtSetpointStatus();
  } 
 
}

