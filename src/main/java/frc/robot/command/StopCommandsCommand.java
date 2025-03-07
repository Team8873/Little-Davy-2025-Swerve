// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
/** An  command that uses an  subsystem. */
public class StopCommandsCommand extends Command {
  private final Intake m_subsystem;
  private final Elevator m_elevator;
  private final Arm m_arm;
  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopCommandsCommand(Elevator elevator, Intake intake, Arm arm) {
    m_subsystem = intake;
    m_arm = arm;
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake,arm,elevator);
    getInterruptionBehavior();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
