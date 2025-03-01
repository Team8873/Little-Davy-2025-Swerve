// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightFace;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

/** An  command that uses an  subsystem. */
public class limeLightLock extends Command {

  private final LimeLightFace m_lime;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric m_drive;
  private final SwerveRequest.RobotCentric m_forwardStraight;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public limeLightLock(LimeLightFace limelight, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.RobotCentric forwardStraight) {
    m_lime = limelight;
    m_drivetrain=drivetrain; 
    m_drive = drive;
    m_forwardStraight = forwardStraight;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight,drivetrain);
    getInterruptionBehavior();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.applyRequest(() ->
        m_drive.withRotationalRate(m_lime.limelight_aim_proportional()));
    m_drivetrain.applyRequest(() ->
        m_forwardStraight.withVelocityX(m_lime.limelight_range_proportional()* 0.001));
    
    
       

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
