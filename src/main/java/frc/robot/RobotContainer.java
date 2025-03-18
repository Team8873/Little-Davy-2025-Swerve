// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TimeOfFlightSensor;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem.Color;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightFace;
import frc.robot.command.ElevatorPresetCommand;
import frc.robot.command.FlipWrist90Command;
import frc.robot.command.FlipWristCommand;
import frc.robot.command.ArmDockCommand;
import frc.robot.command.CANdleAnimationCommand;
import frc.robot.command.ActiveHumanIntakeCommand;
import frc.robot.command.DockHumanIntakeCommand;
import frc.robot.command.PresetAutoCommand;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                              // top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation
                                                                                                  // per second max
                                                                                                  // angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 2% deadband
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
    public final LimeLightFace limeLightFace = new LimeLightFace();
    public final CANdleSystem caNdleSystem = new CANdleSystem(joystick);
    public final Climber epicClimber = new Climber();
    public final LimelightHelpers limelightHelp = new LimelightHelpers();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        elevator.setFollower();
        CameraServer.startAutomaticCapture();
        CameraServer.startAutomaticCapture();

        // NamedCommands.registerCommand("Elevator lvl4", new ElevatorPresetCommand(elevator,arm,'y',false));
        NamedCommands.registerCommand("Hug", new ArmDockCommand(arm));
        new EventTrigger("Elevator lvl4")
        .onTrue(new ElevatorPresetCommand(elevator, arm, 'y', false).withTimeout(5)
        .andThen(new FlipWristCommand(arm).withTimeout(1)
        .andThen((arm.kcikArm()).withTimeout(3)
        .andThen(new ElevatorPresetCommand(elevator, arm, 't', false)
        ))));
        // new EventTrigger("Elevator lvl3")
        // .onTrue(new ElevatorPresetCommand(elevator, arm, 'x', false).withTimeout(5)
        // .andThen((intake.intakeEject()).withTimeout(2)
        // .andThen(new ElevatorPresetCommand(elevator, arm, 't', false))));
        // new EventTrigger("Elevator lvl2")
        // .onTrue(new ElevatorPresetCommand(elevator, arm, 'b', false).withTimeout(5)
        // .andThen((intake.intakeEject().withTimeout(2)
        // .andThen(new ElevatorPresetCommand(elevator, arm, 't', false)))));
        new EventTrigger("Elevator lvl1")
        .onTrue(new ElevatorPresetCommand(elevator, arm, 'o', true).withTimeout(3)
        .andThen(new WaitCommand(1)
        .andThen(intake.intakeEject().withTimeout(1)
        .andThen(new ElevatorPresetCommand(elevator, arm, 'u', true)
        .andThen(intake.stopIntake().withTimeout(.1)
        )))));
        new EventTrigger("Hug").onTrue(NamedCommands.getCommand("Hug"));
        new EventTrigger("standCoral").onTrue(intake.runIntake().alongWith(arm.standArm()).withTimeout(3));
        caNdleSystem.startAnimation();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        // tOFSensor.coralInRange.whileTrue(intake.holdIntake());

        // operator.rightTrigger(.2).and(tOFSensor.coralInRange).onTrue(new
        // IntakeEjectCommand(intake, tOFSensor).andThen(new
        // ElevatorPresetCommand(elevator, intake, arm, 'a', false)));

        intake.setDefaultCommand(intake.moveIntake(operator));

        arm.setDefaultCommand(arm.moveArm(operator));
        // tOFSensor.setDefaultCommand(tOFSensor.getDistance());
        elevator.setDefaultCommand(elevator.moveElevator(operator));
        // operator.y().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'y', false).withTimeout(4)
        //         .andThen(new PresetAutoCommand(elevator, arm, intake,0))); //lvl4

        limeLightFace.hasTarget.onTrue(caNdleSystem.ledAnimation(1, Color.Green, AnimationTypes.Strobe));
        limeLightFace.hasTarget.onFalse(caNdleSystem.ledAnimation(1, Color.Green, AnimationTypes.ColorFlow));


        operator.a().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'a', true).withTimeout(5)); //lvl1
        operator.b().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'b', false).withTimeout(5)); //lvl2
        operator.x().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'x', false).withTimeout(5)); //lvl3
        operator.y().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'y', false).withTimeout(5)); //lvl4
        operator.pov(90).whileTrue(new ElevatorPresetCommand(elevator, arm, 'h', false).withTimeout(5)); //lvl4humancoralintake
        //operator.pov(270).whileTrue(new ElevatorPresetCommand(elevator, arm, 'h', true).withTimeout(5)); //humanintakecoral
        operator.pov(270).whileTrue(new ElevatorPresetCommand(elevator, arm, 's', false).withTimeout(5)); //humanintakecoral
        operator.pov(180).whileTrue(new ElevatorPresetCommand(elevator, arm, 'g', true).withTimeout(5)); //groundpickcoral
        operator.leftBumper().whileTrue(new ElevatorPresetCommand(elevator, arm, 'q', true).withTimeout(5)); //groundpickaglae
        operator.rightBumper().whileTrue(caNdleSystem.ledAnimation(0, Color.Pink, AnimationTypes.ColorFlow));

        // operator.pov(180).whileTrue(new ElevatorPresetCommand(elevator, arm, 's',
        // false).withTimeout(5));

        operator.button(9).onTrue(new FlipWristCommand(arm).withTimeout(1));
        operator.button(10).onTrue(new FlipWrist90Command(arm).withTimeout(1));

        // climber stuff:
        joystick.y().whileTrue(epicClimber.engageServo().withTimeout(0.2).andThen(epicClimber.letGoOfCage()));
        joystick.leftTrigger(0.03).whileTrue(epicClimber.disengageServo().withTimeout(0.2).andThen(epicClimber.grabCage(joystick)));
        joystick.leftTrigger(0.1).onFalse(epicClimber.dontMoveClimberDown());
        joystick.y().onFalse(epicClimber.dontMoveClimberDown());



        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrain.applyRequest(() -> {
                    // double DriveBoost = 0.5 + (joystick.getRightTriggerAxis() * 0.5);
                    double startboost = 0.2;
                    double endboost = elevator.getElevatorPos() > 1.5 ? .5 : 1 ;
                    double joytrigR = joystick.getRightTriggerAxis();
                    double lerpboost = startboost * (1.0 - joytrigR) + endboost * joytrigR;
                //     double startcoral = 1;
                //     double endcoral = 2;
                //     double joytrigL = joystick.getLeftTriggerAxis();
                //     double lerpcoral = startcoral * (1.0 - joytrigL) + endcoral * joytrigL;

                    return drive.withVelocityX((-joystick.getLeftY() * MaxSpeed) * (lerpboost)) // Drive forward with
                                                                                           // negative Y (forward)
                            .withVelocityY((-joystick.getLeftX() * MaxSpeed) * (lerpboost)) // Drive left with negative X
                                                                                       // (left)
                            .withRotationalRate((-joystick.getRightX() * MaxAngularRate) * (lerpboost)); // Drive
                                                                                                    // counterclockwise
                                                                                                    // with negative
                                                                                                    // X (left)
                }));

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        joystick.leftStick().whileTrue(Travel());
        joystick.rightStick().whileTrue(lockOnCommand());
        
        joystick.rightBumper().whileTrue(magicLimeRight());
        joystick.leftBumper().whileTrue(magicLimeLeft());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.pov(90).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

    }
    public Command lockOnCommand() {
        return drivetrain
                .applyRequest(() -> drive.withRotationalRate((limeLightFace.alignRobot(drivetrain.getState().RawHeading.getRadians()))*2));
    }
    public Command Travel() {
        return drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(limeLightFace.limelight_range_proportional())
                .withVelocityY(limeLightFace.limelight_aim_proportional()*0.01));
    }
    public Command magicLimeRight(){
        return drivetrain.applyRequest(()-> {
                return forwardStraight
                        .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getRadians()))
                        .withVelocityX(limeLightFace.limelight_range_proportional())
                        .withVelocityY(limeLightFace.limelight_right_strafe_proportional());
        });
    }
    public Command magicLimeLeft(){
        return drivetrain.applyRequest(()-> {
                return forwardStraight
                        .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getRadians()))
                        .withVelocityX(limeLightFace.limelight_range_proportional())
                        .withVelocityY(limeLightFace.limelight_left_strafe_proportional(drivetrain.getState().Pose));
        });
    }
//     public SequentialCommandGroup Autolvl4() {
//         return
//                 new ElevatorPresetCommand(elevator,arm,'y',false)
//                         .andThen(new PresetAutoCommand(elevator, arm, intake, 0)
//                                 );
//     }
    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
