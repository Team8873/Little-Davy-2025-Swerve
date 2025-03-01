package frc.robot.subsystems;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.LimelightHelpers;

public class LimeLightFace extends SubsystemBase{

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.   //New from ctre github
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3); //
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3); //
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);    //
  private final LimeLightFace m_swerve = new LimeLightFace();                     //New from Ctre github set to robot container

  private final XboxController joystick = XboxController();
  //double getPeriod=0;  -> trying to solve error in last line


// simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = 0.035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= RobotContainer.MaxAngularRate; //from drivetrain.kmaxangularspeed

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = 10;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= RobotContainer.MaxSpeed; //from drivetrain.kmaxspeed
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftY(), 0.02))
            * RobotContainer.MaxSpeed; //from drivetrain.kmaxangularspeed

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftX(), 0.02))
            * RobotContainer.MaxSpeed; //from drivetrain.kmaxangularspeed

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getRightX(), 0.02))
            * RobotContainer.MaxAngularRate; //from drivetrain.kmaxangularspeed

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    //joystick.rightBumper().whileTrue( joystickrightbumper = 1);

    if(joystick.getRightBumperButtonPressed())
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }
/**
     * @param operator the joystick to read from
     * @return the action/method to run
     */
    public Command moveElevator(CommandXboxController joystick){
      return this.run(
          () -> {
          readFromController(operator);
          });
    //m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
 
}