package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Intake extends SubsystemBase {
  private double speed = 0;

  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);

  private double velocityRPM = 0;
  // create RelativeEncoder object
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  // creates shuffleboard stuff
  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private GenericEntry speedEntry = tab.add("Intake Motor", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(0, 2)
      .getEntry();

  private GenericEntry outputWid = tab.add("Intake Output", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(4, 2)
      .getEntry();
  private GenericEntry velocityEntry = tab.add("Intake Velocity", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(5, 2)
      .getEntry();

  public Intake() {
    tab.addDouble("Intake Current", () -> intakeMotor.getOutputCurrent()).withPosition(1, 1).withWidget(BuiltInWidgets.kNumberBar);
  }
  // private ComplexWidget pidEntry =
  // tab.add("Intake Pid", velocityPid)
  // .withWidget(BuiltInWidgets.kPIDController)
  // .withPosition(6,1);

  /**
   * sets speed to 1
   * 
   * @return the action to run
   */
  public Command runIntake() {
    return this.run(
        () -> {
          speed = -1;
          setSpeed();
        });
  }

  /**
   * sets speed to 0
   * 
   * @return the action to run
   */
  public Command stopIntake() {
    return this.runOnce(
        () -> {
          speed = 0;

          setSpeed();
        });
  }

  /**
   * sets speed 0.15
   * 
   * @return references lastest object(intake) and runs the motor
   */
  public Command holdIntake() {
    speed = 0.15;
    return this.run(
        () -> {
          setSpeed();
        });
  }

  /**
   * sets speed to -1
   * 
   * @return references latest object(intake) and runs the motor
   */
  public Command intakeEject() {
    return this.run(() -> {
      speed = 1;
      setSpeed();
    });

  }

  public Command moveIntake(CommandXboxController operator) {
    return this.run(
        () -> {
          speed = operator.getLeftTriggerAxis() - operator.getRightTriggerAxis() - .06;
          setSpeed();
        });
  }

  /**
   * sets the speed of the intake motor
   */
  private void setSpeed() {
    intakeMotor.set(speed);
  }

  private void getVelocity() {
    velocityRPM = intakeEncoder.getVelocity();
  }

  // things that get called every 20 ms
  @Override
  public void periodic() {
    getVelocity();
    updateShuffleboard();
  }

  private void updateShuffleboard() {
    speedEntry.setDouble(speed);
    outputWid.setDouble(intakeMotor.getAppliedOutput());
    velocityEntry.setDouble(velocityRPM);

  }
}
