
//aryan only
//modeled after elevator code
//don't worry about unused imports. Imported them because I thought I needed them, but there is a lot of overlap so you don't use all

package frc.robot.subsystems;

//spark max imports
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//switched to throughbore so don't need

//command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//controller import: dont need because controller stuff is in robot container but leave for now just in case
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//imports constants from Constants.java
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.epilogue.Logged;
//pid import
import edu.wpi.first.math.controller.PIDController;
//servo import
import edu.wpi.first.wpilibj.Servo;

@Logged
public class Climber extends SubsystemBase { // puts climber as a subsystem; inside is code for the climber
    // introduce stuff
    private final SparkMax motorForClimber = new SparkMax(ClimberConstants.motorForClimberCANID, MotorType.kBrushless);                                                                                                                  // motorForClimber
    // private final DutyCycleEncoder encoderForClimber = new DutyCycleEncoder(ClimberConstants.encoderForClimberDIOPort); 
    private final Servo climberServo = new Servo(ClimberConstants.servoID); // a servo called climberServo

    // its not at the setpoint when we turn it on

    private double servoValue = 0;
    // shuffleboard entries

    // SERVOSTUFF
    // need to trip the servo to move the motor in positive direction. 0.5 is
    // engaged 0.0 is disengaged
    // use 1.0 for positive direction and 0.0 for negative. It's location but this
    // how to use it. It might be flipped around, don't know until test

    /**
     * engages the servo for climber
     * @return 
     */
    public Command engageServo() {
        return this.run(
                () -> {
                    servoValue = 1.0;
                    climberServo.set(servoValue);
                });
    }

    /**
     * Disengages climber servo
     * @return
     */
    public Command disengageServo() {
        return this.run(
                () -> {
                    servoValue = 0.0;
                    climberServo.set(servoValue);
                });
    }

    public Climber() {
    }

    /**
     * Releases cage
     * @return
     */
    public Command letGoOfCage() {
        return this.run(
                () -> {
                    // speed = climberPid.calculate(climberMotorPosition);
                    // disengageServo
                    // setSpeed();
                    motorForClimber.set(0.7
                    );


                });
    }

    /**
     * Engages the climber with the cage
     * @param joystick controller to read inputs from 
     * @return actions to run
     */
    public Command grabCage(CommandXboxController joystick) {
        return this.run(
                () -> {
                    double climberSpeed = joystick.getLeftTriggerAxis();
                    motorForClimber.set(-climberSpeed);
                    // engageServo

                });
    }

    /**
     * stops the climber motor 
     * @return action to run
     */
    public Command dontMoveClimberDown() {
        return this.runOnce(
                () -> {
                    motorForClimber.set(0);
                });
    }

}