package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
  private GenericEntry posewid = tab.add("distance From apriltag", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(7, 3)
      .getEntry();
  private GenericEntry sped = tab.add("veloc", 0)
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 4)
      .getEntry();
  private PIDController rotationPid = new PIDController(4, 0, 0);
  private PIDController velocityPid = new PIDController(.04, 0, 0);
  private PIDController forwardPid = new PIDController(.1, 0, 0);

  private RawFiducial[] fiducials;

  private int m_id;
  private boolean hasAprilTagTarget = false;
  private double tx;
  private double ty;
  private double thinkPos;
  private boolean robotAngleFaceCorrect;

  public LimeLightFace() {
    rotationPid.setTolerance(0.01);
    forwardPid.setTolerance(.1);
    velocityPid.setTolerance(0.2);
    rotationPid.enableContinuousInput(-Math.PI, Math.PI);
    tab.addInteger("april tag Id", () -> m_id).withPosition(6, 2);
    tab.addDouble("think pos", () -> thinkPos).withWidget(BuiltInWidgets.kNumberBar).withPosition(6, 3);
  }

  /**
   * Auto face april tag
   * 
   * @return the speed
   */
  public double limelight_aim_proportional() {
    double kP = 0.02;
    double targetingAngularVelocity = tx * kP;
    targetingAngularVelocity *= RobotContainer.MaxAngularRate;
    return targetingAngularVelocity;
  }

  /**
   * Auto drive robot to lvl4 scoring pos
   * 
   * @return the speed to get to the scoring pos
   */
  public double limelight_range_proportional() {
    double targetingForwardSpeed = ty;
    double target;
    if (robotAngleFaceCorrect) {// if robot is not facing the general right way stay a certain distance away
                                // from reef
      target = -6.1;
    } else {
      target = -10;
    }
    double speed = forwardPid.calculate(targetingForwardSpeed, target);
    posewid.setDouble(targetingForwardSpeed);
    if (!hasAprilTagTarget) {
      return 0;
    }
    return speed;
  }

  /**
   * Auto strafe based on tx of april tag
   * Mainly used for lvl4 scoring
   * 
   * @param direction which reef pipe to score on -1 for right 1 for left
   * @return speed to get to the side pos for scoring
   */
  public double limelight_strafe_proportional(int direction) {
    if (!hasAprilTagTarget) { // checks for april tag return speed 0 if not found
      return 0;
    }
    double targetTx;

    if (robotAngleFaceCorrect) { // check if robot is facing the general right direction
      if (ty < -15) {
        targetTx = 0;
      } else {
        switch (direction) {
          case 1:
            targetTx = 15.3;
            break;
          case -1:
            targetTx = -16.7;
            break;
          default:
            targetTx = 0;
            break;
        }
      }
    } else {
      targetTx = thinkPos - Math.pow(getRadianPose(thinkPos), 2); // if not then strafe until it is
    }
    double targetAngleStrafe = tx;
    double speed = velocityPid.calculate(targetAngleStrafe, targetTx);
    sped.setDouble(speed);
    return speed;
  }

  /**
   * Auto aligns robot with reef
   * 
   * @param currentPose the current angle of the robot
   * @return the rotation needed to correct the angle of the robot
   */
  public double alignRobot(double currentPose) {
    if (ty < -15 || !robotAngleFaceCorrect) { // robot is really far away or not facing the general right direction
      return -limelight_aim_proportional() / 2; // faces robot to april tag instead of reef
    }
    double speed = rotationPid.calculate(currentPose, getRadianPose(currentPose));
    wid.setDouble(speed);
    return speed;
  }

  public Command poseGetter(double currentPose) {
    return this.runOnce(
        () -> {
          thinkPos = currentPose;
        });
  }

  public final Trigger hasTarget = new Trigger(() -> hasAprilTagTarget);

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public double getOffset() {
    double offset = Math.abs(tx) - 15 + Math.abs(ty + 6.1);
    return Math.abs(offset);
  }

  /**
   * Gets where robot should be facing based on aprilTag id
   * 
   * @param currentPose the current angle of robot in radians
   * @return the target angle of the robot
   */
  private double getRadianPose(double currentPose) {
    double radianPose;
    switch (m_id) {
      case 18, 14, 15, 7, 5, 4:
        // radianPose = Math.PI;
        radianPose = 0;
        break;
      case 21, 10:
        // radianPose = 0;
        radianPose = Math.PI;
        break;
      case 16, 3:
        radianPose = Math.PI / 2;
        break;
      case 12, 2:
        radianPose = Math.PI / 4;
        break;
      case 13, 1:
        radianPose = -Math.PI / 4;
        break;
      case 20, 11:
        radianPose = Math.PI / 6;
        break;
      case 22, 9:
        radianPose = -Math.PI / 6;
        break;
      case 19, 6:
        radianPose = (5 * Math.PI) / 6;
        break;
      case 17, 8:
        radianPose = -(5 * Math.PI) / 6;
        break;

      default:
        radianPose = currentPose;
        break;
    }
    return radianPose;
  }

  public void periodic() {

    fiducials = LimelightHelpers.getRawFiducials("limelight"); // gets limelight raw output
    if (fiducials.length < 1) { // if limelight no output default state
      hasAprilTagTarget = false;
      ty = 0;
      tx = 0;
    } else {
      hasAprilTagTarget = true;
      m_id = fiducials[0].id;
      ty = LimelightHelpers.getTY("limelight");
      tx = LimelightHelpers.getTX("limelight");
      robotAngleFaceCorrect = MathUtil.isNear(getRadianPose(thinkPos), thinkPos, Units.degreesToRadians(15));
    }

  }

}
