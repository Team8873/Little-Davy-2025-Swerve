// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PresetConstants;

/** An  command that uses an  subsystem. */
public class ElevatorPresetCommand extends Command {
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Arm m_arm;
  private double wristPos = PresetConstants.wristFlatPos;
  private double armPos = PresetConstants.lvl1to3ArmPos;
  private double elevatorPos = PresetConstants.lvl1Elevator;
  private char m_button_pressed = 'a';
  private boolean m_wristSide = false;
  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorPresetCommand(Elevator elevator, Intake intake, Arm arm, char button, boolean wristSide) {
    m_elevator = elevator;
    m_intake = intake;
    m_arm = arm;
    m_button_pressed = button;
    m_wristSide  = wristSide;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator,intake,arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    checkPresetLvl();
    m_arm.setArmMechTarget(wristPos, armPos);
    m_elevator.targetPosition(elevatorPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.elevatorPreset();
    m_arm.armPreset();
    m_arm.wristPreset().onlyIf(m_arm.getArmSetpointStatus());
    m_intake.intakeEject().onlyIf(m_elevator.elevatorAtTarget.and(m_arm.armMechAtTarget));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getElevatorSetpointStatus().getAsBoolean() && 
    (m_arm.getArmSetpointStatus().getAsBoolean() && m_arm.getWristSetpointStatus().getAsBoolean());
  }

  private void checkPresetLvl(){
    if(m_wristSide){
      armPos = PresetConstants.lvl1to3ArmPosSide;
      switch (m_button_pressed) {
        case 'a': elevatorPos = PresetConstants.lvl1ElevatorSide;
          break;
        case 'b': elevatorPos = PresetConstants.lvl2ElevatorSide;
          break;
        case 'x': elevatorPos = PresetConstants.lvl3ElevatorSide;
          break;
      }}else{
        switch (m_button_pressed) {
      case 'b': elevatorPos = PresetConstants.lvl2Elevator;
        break;
      case 'x': elevatorPos = PresetConstants.lvl3Elevator;
        break;
      case 'y': elevatorPos = PresetConstants.lvl4Elevator;
                armPos = PresetConstants.lvl4ArmPos;
        break;
        }
      }

  }

}
