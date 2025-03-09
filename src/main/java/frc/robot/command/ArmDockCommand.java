// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TimeOfFlightSensor;
import edu.wpi.first.wpilibj2.command.Command;

/** An  command that uses an  subsystem. */
public class ArmDockCommand extends Command {
  //private final Intake m_intake;
  private final Arm m_arm;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmDockCommand(Arm arm) {
    //m_intake = intake;
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    getInterruptionBehavior();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmMechTarget(PresetConstants.wristDockPos,PresetConstants.humanIntakeArmPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_arm.setArmSpeed();
    m_arm.setWristSpeed(); // fix wrist thing
    //m_intake.runIntake().onlyIf(m_arm.armMechAtTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.armMechAtTarget.getAsBoolean();
  }
}
