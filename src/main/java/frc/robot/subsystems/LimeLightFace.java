package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.Runtime;

public class LimeLightFace extends SubsystemBase{

    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private GenericEntry wid =
      tab.add("Rotation rate", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(6,1)
         .getEntry();
    private GenericEntry radwid =
      tab.add("april tag Id", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(6,2)
         .getEntry();
         private GenericEntry poswid =
      tab.add("think pos", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(6,3)
         .getEntry();
         private GenericEntry sped =
      tab.add("veloc", 0)
         .withWidget(BuiltInWidgets.kNumberBar)
         .withPosition(6,4)
         .getEntry();
    private PIDController rotationPid = new PIDController(4, 0, 0);
    private PIDController velocityPid = new PIDController(.05, 0, 0);
    private PIDController forwardPid = new PIDController(.1, .01, 0);

    private RawFiducial[] fiducials;
  public LimeLightFace(){
    rotationPid.setTolerance(0.005);
    forwardPid.setTolerance(.3);
  }
//   Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.   //New from ctre github
//  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3); //
//  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3); //
//  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);    //
//  private final LimeLightFace m_swerve = new LimeLightFace();                     //New from Ctre github set to robot container


//   double getPeriod=0;  -> trying to solve error in last line


// simple proportional turning control with Limelight.
//   "proportional control" is a control algorithm in which the output is proportional to the error.
//   in this case, we are going to return an angular velocity that is proportional to the 
//   "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = 0.02;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= RobotContainer.MaxAngularRate; //from drivetrain.kmaxangularspeed

    //invert since tx is positive when the target is to the right of the crosshair
    //targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double target = -6.1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight");
    double speed = forwardPid.calculate(targetingForwardSpeed, target);
    return speed;
  }

  //private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
   // var xSpeed =
     //   -m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftY(), 0.02))
     //       * RobotContainer.MaxSpeed; //from drivetrain.kmaxangularspeed

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    //var ySpeed =
     //   -m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftX(), 0.02))
      //      * RobotContainer.MaxSpeed; //from drivetrain.kmaxangularspeed

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    //var rot =
      //  -m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getRightX(), 0.02))
      //      * RobotContainer.MaxAngularRate; //from drivetrain.kmaxangularspeed

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    //joystick.rightBumper().whileTrue( joystickrightbumper = 1);

   // if(joystick.getRightBumperButtonPressed())
   // {
     //   final var rot_limelight = limelight_aim_proportional();
    //    rot = rot_limelight;

     //   final var forward_limelight = limelight_range_proportional();
     //   xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
     //   fieldRelative = false;}
    //}
    public double limelight_left_strafe_proportional()
  {    
  
    if(fiducials.length < 1){
      return 0;
  }
    double lefttargetTx = 14.7;

    
    double targetAngleStrafe = (LimelightHelpers.getTX("limelight"));
    System.out.println(targetAngleStrafe);
    double speed = velocityPid.calculate(targetAngleStrafe, lefttargetTx);
    sped.setDouble(speed);
    
    return speed;
  }
  public double limelight_right_strafe_proportional()
  {    
    
    if(fiducials.length < 1){
      return 0;
  }
    double righttargetTx = -14.7;

    double targetAngleStrafe = (LimelightHelpers.getTX("limelight"));
    System.out.println(targetAngleStrafe);
  double speed = velocityPid.calculate(targetAngleStrafe, righttargetTx);
    sped.setDouble(speed);

    return speed;
  }
private int m_id;
  public double alignRobot(double currentPose){
     if(fiducials.length < 1){
      return 0;
  }
  m_id = fiducials[0].id;
  int aprilTagID = m_id;
      radwid.setInteger(aprilTagID);
      poswid.setDouble(currentPose);
  double radianPose;
  switch (aprilTagID) {
      case 18, 14, 15, 7, 5, 4:
          radianPose = 0;
          // radianPose = 2* Math.PI;
          break;
      case 21, 10:
          radianPose = Math.PI;
          // radianPose = 2* Math.PI;
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
  double speed = rotationPid.calculate(currentPose, radianPose);
  
      wid.setDouble(speed);
        
      
  return speed;


  }
  public void periodic(){
   fiducials = LimelightHelpers.getRawFiducials("limelight");

  }
  
    //m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
}