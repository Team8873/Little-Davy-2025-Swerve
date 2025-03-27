package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class LimeLightFace extends SubsystemBase {

  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  private GenericEntry wid = tab.add("Rotation rate", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 1)
      .getEntry();
  private GenericEntry radwid = tab.add("april tag Id", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 2)
      .getEntry();
  private GenericEntry poswid = tab.add("think pos", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 3)
      .getEntry();
  private GenericEntry posewid = tab.add("distance From apriltag", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(7, 3)
      .getEntry();
  private GenericEntry sped = tab.add("veloc", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 4)
      .getEntry();

  private PIDController rotationPid = new PIDController(0.25, 0.0, 0);
  private PIDController velocityPid = new PIDController(.04, 0, 0);
  private PIDController forwardPid = new PIDController(.1, 0, 0);
  private ComplexWidget pidwid = tab.add("speed pid", forwardPid).withWidget(BuiltInWidgets.kPIDController);
  private ComplexWidget rpidwid = tab.add("rotation pid", rotationPid).withWidget(BuiltInWidgets.kPIDController);

  private RawFiducial[] fiducials;

  private int m_id;
  private boolean hasAprilTagTarget = false;
  private double tx;
  private double ty;

  public LimeLightFace() {
    rotationPid.setTolerance(0.01);
    forwardPid.setTolerance(.1);
    velocityPid.setTolerance(0.2);
    rotationPid.enableContinuousInput(-180, 180);
  }

  public double limelight_aim_proportional() {
    double kP = 0.02;
    double targetingAngularVelocity = tx * kP;
    targetingAngularVelocity *= RobotContainer.MaxAngularRate;
    return targetingAngularVelocity;
  }

  public double limelight_range_proportional() {
    double target = -6.1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight");
    double speed = forwardPid.calculate(targetingForwardSpeed, target);
    posewid.setDouble(targetingForwardSpeed);
    if (!hasAprilTagTarget) {
      return 0;
    }
    return speed;
  }

  public double limelight_left_strafe_proportional() {
    if (!hasAprilTagTarget) {
      return 0;
    }
    double lefttargetTx;
    if (LimelightHelpers.getTY("limelight") < -15) {
      lefttargetTx = 0;
    } else {
      lefttargetTx = 15.3;
    }
    double targetAngleStrafe = (tx);
    System.out.println(targetAngleStrafe);
    double speed = velocityPid.calculate(targetAngleStrafe, lefttargetTx);
    sped.setDouble(speed);

    return speed;
  }

  public double limelight_right_strafe_proportional() {
    if (!hasAprilTagTarget) {
      return 0;
    }

    double righttargetTx;
    if (LimelightHelpers.getTY("limelight") < -15) {
      righttargetTx = 0;
    } else {
      righttargetTx = -16.7;
    }
    double targetAngleStrafe = (tx);
    System.out.println(targetAngleStrafe);
    double speed = velocityPid.calculate(targetAngleStrafe, righttargetTx);
    sped.setDouble(speed);

    return speed;
  }

  public double alignRobot(double currentPose) {
    if (ty < -15) {
      return -limelight_aim_proportional() / 2;
    }

    radwid.setInteger(m_id);
    poswid.setDouble(currentPose);
    double radianPose;
    switch (m_id) {
      case 18, 14, 15, 7, 5, 4:
        radianPose = 180;
        // radianPose = 0;
        break;
      case 21, 10:
        radianPose = 0;
        // radianPose = Math.PI;
        break;
      case 16, 3:
        radianPose = 90;
        break;
      case 12, 2:
        radianPose = 45;
        break;
      case 13, 1:
        radianPose = -45;
        break;
      case 20, 11:
        radianPose = 60;
        break;
      case 22, 9:
        radianPose = -60;
        break;

      case 19, 6:
        radianPose = 120;
        break;
      case 17, 8:
        radianPose = -120;
        break;

      default:
        radianPose = currentPose;
        break;
    }
    double speed = rotationPid.calculate(currentPose, radianPose);
    wid.setDouble(speed);
    return speed;
  }

  public Command poseGuesser(double currentPose) {
    return this.run(
        () -> {
          LimelightHelpers.SetRobotOrientation("limelight", currentPose, 0, 0, 0, 0, 0);

        });
  }

  public final Trigger hasTarget = new Trigger(() -> hasAprilTagTarget);
  public double getTx(){
    return tx;
  }

  public double getTy(){
    return ty;
  }
  public double getOffset(){
    double offset = Math.abs(tx) - 15 + Math.abs(ty + 6.1);
    return Math.abs(offset);
  }
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("limelight");
    if (fiducials.length < 1) {
      hasAprilTagTarget = false;
      ty = 0;
      tx = 0;
    } else {
      hasAprilTagTarget = true;
      m_id = fiducials[0].id;
      ty = LimelightHelpers.getTY("limelight");
      tx = LimelightHelpers.getTX("limelight");
    }

  }

}