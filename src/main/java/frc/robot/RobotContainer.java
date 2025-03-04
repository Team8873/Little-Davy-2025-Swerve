// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;



import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TimeOfFlightSensor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightFace;
import frc.robot.command.ElevatorPresetCommand;
import frc.robot.command.FlipWrist90Command;
import frc.robot.command.FlipWristCommand;
import frc.robot.command.ArmDockCommand;
import frc.robot.command.ActiveHumanIntakeCommand;
import frc.robot.command.DockHumanIntakeCommand;


public class RobotContainer {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
     // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Arm arm = new Arm();
    public final Intake intake = new Intake();
    public final TimeOfFlightSensor tOFSensor = new TimeOfFlightSensor();
    public final Elevator elevator = new Elevator();
    public final Climber climber = new Climber();
    public final LimeLightFace limeLightFace  = new LimeLightFace();
    //public final LimeLightFace limeLightFace = new LimeLightFace();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        elevator.setFollower();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        
        intake.setDefaultCommand(intake.moveIntake(operator));

        //If sensor detects something close holds
        //tOFSensor.coralInRange.whileTrue(intake.holdIntake());//commented this out w/ the threshold change
        
        //operator.rightTrigger(.2).and(tOFSensor.coralInRange).onTrue(new IntakeEjectCommand(intake, tOFSensor).andThen(new ElevatorPresetCommand(elevator, intake, arm, 'a', false)));

        arm.setDefaultCommand(arm.moveArm(operator));
        tOFSensor.setDefaultCommand(tOFSensor.getDistance());
        elevator.setDefaultCommand(elevator.moveElevator(operator));
        
        operator.a().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'a', false).withTimeout(5));
        operator.b().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'b', false).withTimeout(5));
        operator.x().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'x', false).withTimeout(5));
        operator.y().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'y', false).withTimeout(5));
        operator.pov(270).debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 't', false).withTimeout(5));
        operator.pov(0).debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'h', true).withTimeout(5));

        // operator.leftBumper().negate().and(operator.y().onTrue(new ElevatorPresetCommand(elevator, intake, arm, 'y', false)));

        // operator.leftBumper().and(operator.a().onTrue(new ElevatorPresetCommand(elevator, intake, arm, 'a', true)));
        // operator.leftBumper().and(operator.b().onTrue(new ElevatorPresetCommand(elevator, intake, arm, 'b', true)));
        // operator.leftBumper().and(operator.x().onTrue(new ElevatorPresetCommand(elevator, intake, arm, 'x', true)));

        //operator.pov(0).onTrue(new StopCommandsCommand(elevator, intake, arm));

        // operator.back().toggleOnTrue(new DockHumanIntakeCommand(intake));
        // operator.back().toggleOnFalse(new ActiveHumanIntakeCommand(intake));
         operator.button(9).onTrue(new FlipWristCommand(arm).withTimeout(5));
         operator.button(10).onTrue(new FlipWrist90Command(arm).withTimeout(3));
        
        //operator.pov(180).onTrue(new ElevatorPresetCommand(elevator, intake, arm, 'g', false));

        //climber stuff:
        joystick.y().onTrue(climber.moveToEngaged());
        joystick.x().whileTrue(climber.moveClimberDown());
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_xspeedLimiter.calculate((-joystick.getLeftY() * MaxSpeed)*(0.5+(joystick.getRightTriggerAxis()*0.5)))) // Drive forward with negative Y (forward)
                     .withVelocityY(m_yspeedLimiter.calculate((-joystick.getLeftX() * MaxSpeed)*(0.5+(joystick.getRightTriggerAxis()*0.5)))) // Drive left with negative X (left)
                     .withRotationalRate(m_rotLimiter.calculate((-joystick.getRightX() * MaxAngularRate)*(0.5+(joystick.getRightTriggerAxis()*0.5)))) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.rightStick().whileTrue(lockOnCommand()
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(()->
            forwardStraight.withVelocityX(0).withVelocityY(limeLightFace.limelight_right_strafe_proportional() ))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(()->
            forwardStraight.withVelocityX(0).withVelocityY(limeLightFace.limelight_left_strafe_proportional() )));
            

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }
    public SequentialCommandGroup lockOnCommand(){
           return new SequentialCommandGroup(drivetrain.applyRequest(() ->
           drive.withRotationalRate(limeLightFace.limelight_aim_proportional())).withTimeout(.5)
           .andThen(
               drivetrain.applyRequest(()-> 
               forwardStraight.withVelocityX(limeLightFace.limelight_range_proportional() * 0.005))));
    }
    public SequentialCommandGroup strafeRightCommand(){
        return new SequentialCommandGroup(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(limeLightFace.limelight_right_strafe_proportional() * 0.01)).withTimeout(.5)
    );
 }
 public SequentialCommandGroup strafeLeftCommand(){
    return new SequentialCommandGroup(drivetrain.applyRequest(() ->
    forwardStraight.withVelocityX(0).withVelocityY(limeLightFace.limelight_left_strafe_proportional() * 0.01)).withTimeout(.5)
    );
 }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}


