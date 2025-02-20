// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;
import frc.robot.subsystems.Intake;

import frc.robot.Constants;
import frc.robot.Constants.PresetConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An  command that uses an  subsystem. */
public class ActiveHumanIntakeCommand extends Command {
  private final Intake m_intake;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ActiveHumanIntakeCommand(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    getInterruptionBehavior();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.humanTargetPosition(PresetConstants.humanIntakeActivePos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.moveHumanMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.humanIntakeAtPos.getAsBoolean();
  }
}
