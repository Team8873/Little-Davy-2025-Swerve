// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.cameraserver.CameraServer;

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
    public final Elevator elevator = new Elevator();
    public final LimeLightFace limeLightFace = new LimeLightFace();
    public final CANdleSystem caNdleSystem = new CANdleSystem(joystick);
    public final Climber epicClimber = new Climber();
    public final LimelightHelpers limelightHelp = new LimelightHelpers();
    public final TimeOfFlightSensor tOF = new TimeOfFlightSensor();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    // private final SendableChooser<String> alignmentChooser;

        private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");


    public RobotContainer(){
        autoChooser = AutoBuilder.buildAutoChooser("blue left wall to reef front");
        tab.add("Auto Mode", autoChooser).withPosition(5,0).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser); //puts the autons to shuffleboard
        configureBindings(); //creates the triggers
        elevator.setFollower(); //sets follower motor for elevator
        CameraServer.startAutomaticCapture(); //gets video data from cameras
        CameraServer.startAutomaticCapture();
        autoCommands(); //creates auto commands
        caNdleSystem.startAnimation(); //starts up animations
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        intake.setDefaultCommand(intake.moveIntake(operator)); //controls intake with controller

        arm.setDefaultCommand(arm.moveArm(operator));//controls arm with controller
        elevator.setDefaultCommand(elevator.moveElevator(operator)); // controls elevator with controller 

        limeLightFace.hasTarget.onTrue(caNdleSystem.ledAnimation(1, Color.Green, AnimationTypes.SingleFade)); //crossbar flashes green when apriltag seen
        limeLightFace.hasTarget.onFalse(caNdleSystem.ledAnimation(1, Color.Green, AnimationTypes.ColorFlow)); // reverts to normal when none

        operator.a().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'a', true).withTimeout(5)); //lvl1 preset
        operator.b().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'b', false).withTimeout(5)); //lvl2 preset
        operator.x().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'x', false).withTimeout(5)); //lvl3 preset
        operator.y().debounce(0.3).whileTrue(new ElevatorPresetCommand(elevator, arm, 'y', false).withTimeout(5)); //lvl4 preset
        operator.pov(90).whileTrue(new ElevatorPresetCommand(elevator, arm, 'h', false).withTimeout(5)); //lvl4humancoralintake
        operator.pov(0).whileTrue(new ElevatorPresetCommand(elevator, arm, 'h', true).withTimeout(5)); //humanintakecoral
        operator.pov(270).whileTrue(new ElevatorPresetCommand(elevator, arm, 's', false).withTimeout(5)); //humanintakecoral
        operator.pov(180).whileTrue(new ElevatorPresetCommand(elevator, arm, 'g', true).withTimeout(5)); //groundpickcoral
        operator.leftBumper().whileTrue(new ElevatorPresetCommand(elevator, arm, 'q', true).withTimeout(5)); //groundpickaglae
        // operator.rightBumper().whileTrue(caNdleSystem.ledAnimation(0, Color.Pink, AnimationTypes.ColorFlow)); //does some animation thing idk

        // operator.pov(180).whileTrue(new ElevatorPresetCommand(elevator, arm, 's',
        // false).withTimeout(5));

        operator.button(9).onTrue(new FlipWristCommand(arm).withTimeout(1)); //flips wrist
        operator.button(10).onTrue(new FlipWrist90Command(arm).withTimeout(1)); //flips wrist

        // climber stuff:
        joystick.y().whileTrue(epicClimber.engageServo().withTimeout(0.2).andThen(epicClimber.letGoOfCage())); //releases cage
        joystick.leftTrigger(0.03).whileTrue(epicClimber.disengageServo().withTimeout(0.2).andThen(epicClimber.grabCage(joystick))); //grabs cage based on controller trigger

        joystick.leftTrigger(0.1).onFalse(epicClimber.dontMoveClimberDown()); //stops the climber
        joystick.y().onFalse(epicClimber.dontMoveClimberDown()); //stops the climber

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
        
        // joystick.rightBumper().whileTrue(magicLimeRight());
        joystick.rightBumper().and(joystick.x()).whileTrue(magicLimeRight(false)); //auto align lvl 2-3
        joystick.leftBumper().and(joystick.x()).whileTrue(magicLimeLeft(false));

        joystick.rightBumper().and(joystick.x().negate()).whileTrue(magicLimeRight(true)); // auto align lvl4
        joystick.leftBumper().and(joystick.x().negate()).whileTrue(magicLimeLeft(true));

        // joystick.leftBumper().whileTrue(magicLimeLeft());

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
    /**
     * Automatically makes the robot faces the april tag
     * @return the command to do so
     */
    public Command lockOnCommand() {
        return drivetrain
                .applyRequest(() -> drive.withRotationalRate((limeLightFace.alignRobot(drivetrain.getState().Pose.getRotation().getDegrees()))*2));
    }
    /**
     * Automatically drives the robot forward
     * @return the command to do so
     */
    public Command Travel() {
        return drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(limeLightFace.limelight_range_proportional(true))
                .withVelocityY(limeLightFace.limelight_aim_proportional()*0.01));
    }
    /**
     * Auto aligns robot to the right
     * @param lvl4 if you want lvl4
     * @return the command to do so
     */
    public Command magicLimeRight(boolean lvl4){
        return drivetrain.applyRequest(()-> {
                return forwardStraight
                        .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getDegrees()))
                        .withVelocityX(limeLightFace.limelight_range_proportional(lvl4))
                        .withVelocityY(limeLightFace.limelight_right_strafe_proportional());
        });
    }
    /**
     * Auto aligns robot to the left
     * @param lvl4 if you want lvl4
     * @return the command to do so
     */
    public Command magicLimeLeft(boolean lvl4){
        return drivetrain.applyRequest(()-> {
                return forwardStraight
                        .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getDegrees()))
                        .withVelocityX(limeLightFace.limelight_range_proportional(lvl4))
                        .withVelocityY(limeLightFace.limelight_left_strafe_proportional());
        });
    }
    // public Command magicLimeLeftBack(){
    //     return drivetrain.applyRequest(()-> {
    //             return forwardStraight
    //                     .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getDegrees()))
    //                     .withVelocityX(limeLightFace.limelight_backUp())
    //                     .withVelocityY(limeLightFace.limelight_left_strafe_proportional());
    //     });
    // }
    // public Command magicLimeRightBack(){
    //     return drivetrain.applyRequest(()-> {
    //             return forwardStraight
    //                     .withRotationalRate(limeLightFace.alignRobot(drivetrain.getState().RawHeading.getDegrees()))
    //                     .withVelocityX(limeLightFace.limelight_backUp())
    //                     .withVelocityY(limeLightFace.limelight_right_strafe_proportional());
    //     });
    // }
    public Command backUp(){
        return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0));
    }
    
/**
 * PathPlanner event markers and commands for auton
 */
    private void autoCommands() {
        // new EventTrigger("autoRight").onTrue(Commands.runOnce(()-> PPHolonomicDriveController.overrideXFeedback(() -> {return 0.0;}))
        // .alongWith());
        
        ParallelCommandGroup scoreLeft4 = new RepeatCommand(magicLimeLeft(true)).withTimeout(2)
        .alongWith(NamedCommands.getCommand("Elevator lvl4"));

        ParallelCommandGroup getCoral = new ElevatorPresetCommand(elevator, arm, 'i', true).withTimeout(3)
        .alongWith(new RepeatCommand(intake.runIntake())
        // .withTimeout(2)
        .until(tOF.coralInRange)
        );
        
        ParallelCommandGroup scoreRight4 = new RepeatCommand(magicLimeRight(true)).withTimeout(2)
        .alongWith(NamedCommands.getCommand("Elevator lvl4"));

        NamedCommands.registerCommand("Hug", new ArmDockCommand(arm));
        NamedCommands.registerCommand("Elevator lvl4", new ElevatorPresetCommand(elevator, arm, 'y', true).withTimeout(3.5)//changed from 5 sec
        .andThen((arm.kcikArm().onlyIf(tOF.coralInRange)
        ).withTimeout(1)//changed from 1.5
        .andThen(new ElevatorPresetCommand(elevator, arm, 'a', false).withTimeout(1)//bring elevator down
        .andThen(new ElevatorPresetCommand(elevator, arm, 't', false) // go back to travel position
        ))));

        new EventTrigger("Elevator lvl4").onTrue(NamedCommands.getCommand("Elevator lvl4"));

        new EventTrigger("Elevator lvl1")
        .onTrue(new ElevatorPresetCommand(elevator, arm, 'o', true).withTimeout(3)
        .andThen(new WaitCommand(1)
        .andThen(intake.intakeEject().withTimeout(1)
        .andThen(new ElevatorPresetCommand(elevator, arm, 'u', true)
        .andThen(intake.stopIntake().withTimeout(.1)
        )))));

        new EventTrigger("Hug").onTrue(NamedCommands.getCommand("Hug"));
        new EventTrigger("score Left").onTrue(scoreLeft4);
        new EventTrigger("score Right").onTrue(scoreRight4);

        new EventTrigger("humanCoral").onTrue(getCoral
        .andThen(intake.stopIntake().onlyIf(tOF.coralInRange)
        // .andThen(getPathToFollow("humanRight scoreright"))
        ));


        new EventTrigger("standCoral").onTrue(arm.standArm().withTimeout(1)
        .andThen(new RepeatCommand(intake.runIntake()).until(tOF.coralInRange)
        .andThen(intake.stopIntake().onlyIf(tOF.coralInRange)
        .andThen(arm.upArm().withTimeout(.5)))));

        new EventTrigger("go").onTrue(new WaitCommand(2)
        // .andThen(getPathToFollow("score again"))
        );

        new EventTrigger("autoRight").onTrue(new WaitCommand(.1).andThen(new RepeatCommand(magicLimeRight(true)).withTimeout(2)// changed from 2
        .andThen(//new RepeatCommand(magicLimeRightBack().onlyIf(()-> !tOF.coralInRange.getAsBoolean())).withTimeout(2)
            //getPathToFollow("back up front").onlyIf(()-> !tOF.coralInRange.getAsBoolean()) // changed from 2.5
        // .andThen()
        )));

        new EventTrigger("autoRight noPick").onTrue(new WaitCommand(.1).andThen(new RepeatCommand(magicLimeRight(true)).withTimeout(2)// changed from 2
        .andThen(//new RepeatCommand(magicLimeRightBack().onlyIf(()-> !tOF.coralInRange.getAsBoolean())).withTimeout(2)
            //getPathToFollow("back up back").onlyIf(()-> !tOF.coralInRange.getAsBoolean()) // changed from 2.5
        // .andThen(getPathToFollow("back up back"))
        )));

        new EventTrigger("autoLeft").onTrue(new WaitCommand(.1).andThen(new RepeatCommand(magicLimeLeft(true)).withTimeout(2)
        .andThen(//new RepeatCommand(magicLimeLeftBack().onlyIf(()-> !tOF.coralInRange.getAsBoolean())).withTimeout(2)
            //getPathToFollow("back up front").onlyIf(()-> !tOF.coralInRange.getAsBoolean())
        // .andThen(getPathToFollow("rightHuman"))
        )));

        new EventTrigger("human").onTrue(new RepeatCommand(magicLimeLeft(true)).withTimeout(1)
        .andThen(new ElevatorPresetCommand(elevator, arm, 'h', true)
        .andThen(intake.runIntake())));

    }
    public Command getPathToFollow(String pathName){
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        PathConstraints constraints = new PathConstraints(2.0, 1.0,
        Units.degreesToRadians(360), Units.degreesToRadians(45));
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
        return pathfindingCommand;
        } 
        catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
        }

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
