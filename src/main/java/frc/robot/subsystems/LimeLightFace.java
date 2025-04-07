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
import edu.wpi.first.epilogue.Logged;

@Logged
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
  private GenericEntry posewid = tab.add("distance From april", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(7, 3)
      .getEntry();
  private GenericEntry sped = tab.add("veloc", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 4)
      .getEntry();

  private PIDController rotationPid = new PIDController(0.3, 0, 0);
  private PIDController velocityPid = new PIDController(.04, 0, 0);
  private PIDController forwardPid = new PIDController(.1, 0, 0);
  // private ComplexWidget pidwid = tab.add("speed pid",  //tuning pids in shuffleboard
  // forwardPid).withWidget(BuiltInWidgets.kPIDController);
  // private ComplexWidget rpidwid = tab.add("rotation pid",
  // rotationPid).withWidget(BuiltInWidgets.kPIDController);

  private RawFiducial[] fiducials;

  private int m_id;
  private boolean hasAprilTagTarget = false;
  private double tx;
  private double ty;
  private boolean aprilTagChange = false;
  private int previousId = 0;

  public LimeLightFace() {
    rotationPid.setTolerance(0.01);
    forwardPid.setTolerance(.1);
    velocityPid.setTolerance(0.2);
    rotationPid.enableContinuousInput(-180, 180);
  }

  /**
   * look at aprilTag
   * @return speed to go so it looks at it
   */
  public double limelight_aim_proportional() {
    double kP = 0.02;
    double targetingAngularVelocity = tx * kP;
    targetingAngularVelocity *= RobotContainer.MaxAngularRate;
    return targetingAngularVelocity;
  }

  /**
   * Calculates distance from the reef to go
   * @param lvl4 lvl4 distance 
   * @return the speed to go to the reef
   */
  public double limelight_range_proportional(boolean lvl4) {
    double target = -3.8;
    if(!lvl4){
      target = 3;
    }
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight");
    double speed = forwardPid.calculate(targetingForwardSpeed, target);
    posewid.setDouble(targetingForwardSpeed);
    if (!hasAprilTagTarget) {
      return 0;
    }
    return speed;
  }
  // public double limelight_backUp() {
  //   double target = -6.8;
  //   double targetingForwardSpeed = LimelightHelpers.getTY("limelight");
  //   double speed = forwardPid.calculate(targetingForwardSpeed, target);
  //   posewid.setDouble(targetingForwardSpeed);
  //   if (!hasAprilTagTarget) {
  //     return 0;
  //   }
  //   return speed;
  // }

  /**
   * Calculates the speed to strafe the left
   * @return the speed to go to the setpoint
   */
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

  /**
   * Calculates the speed to strafe the right
   * @return the speed to go to the setpoint
   */
  public double limelight_right_strafe_proportional() {
    if (!hasAprilTagTarget) {
      return 0;
    }

    double righttargetTx;
    if (LimelightHelpers.getTY("limelight") < -15) {
      righttargetTx = 0;
    } else {
      righttargetTx = -18.1;
    }
    double targetAngleStrafe = (tx);
    System.out.println(targetAngleStrafe);
    double speed = velocityPid.calculate(targetAngleStrafe, righttargetTx);
    sped.setDouble(speed);

    return speed;
  }

  /**
   * gets the degree to align the robot to the reef with
   * @param currentPose the current pose of the robot
   * @return target pose 
   */
  public double alignRobot(double currentPose) {

    if (ty < -15) {
      return -limelight_aim_proportional() / 2; // keeps target if too far away
    }

    radwid.setInteger(m_id); //shuffleboard
    poswid.setDouble(currentPose);
    double degreePose;
    switch (m_id) {
      case 18, 7:
        degreePose = 180;
        // radianPose = 0;
        break;
      case 21, 10:
        degreePose = 0;
        // radianPose = Math.PI;
        break;
      case 20, 11:
        degreePose = 60;
        break;
      case 22, 9:
        degreePose = -60;
        break;
      case 19, 6:
        degreePose = 120;
        break;
      case 17, 8:
        degreePose = -120;
        break;

      default:
        degreePose = currentPose;
        break;
    }
    double speed = rotationPid.calculate(currentPose, degreePose);
    wid.setDouble(speed);
    return speed;
  }

  // A boolean that gets automatically updated
  // Used in robot container
  public final Trigger hasTarget = new Trigger(() -> hasAprilTagTarget);

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