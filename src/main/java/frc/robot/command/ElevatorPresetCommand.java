// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An  command that uses an  subsystem. */
public class ElevatorPresetCommand extends Command {
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Arm m_arm;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorPresetCommand(Elevator elevator, Intake intake, Arm arm) {
    m_elevator = elevator;
    m_intake = intake;
    m_arm= arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator,intake,arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmTarget(0 ,0);
    m_elevator.targetPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.armPreset();
    m_elevator.elevatorPreset();
    m_intake.intakeEject().onlyIf(m_elevator.elevatorAtTarget.and(m_arm.armAtTarget));
    
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
