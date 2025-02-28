// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    m_elevator = elevator;  //saves a local reference to the elevator subsystem
    m_intake = intake;  //saves a local reference to the intake subsystem
    m_arm = arm;  //saves a local reference to the arm subsystem
    m_button_pressed = button;  //saves a local reference to what button was pressed 
    m_wristSide  = wristSide; //saves a local reference if whether the wirst needs to be on its side or not
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator,arm);
    getInterruptionBehavior();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Presets not set");
    m_elevator.targetPosition(elevatorPos);
    //m_arm.setArmMechTarget(wristPos, armPos);
    checkPresetLvl();
    System.out.print("Presets set");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("running the command");
    m_elevator.elevatorPreset();
    //m_arm.armPreset();
    //while(m_arm.getArmSetpointStatus().getAsBoolean()){m_arm.wristPreset();} //runs the wirst preset ONLY IF the arm is in position
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean();
  }

  /**
   * takes the wristSide boolean and button pressed to determine which preset level to use
   */
  private void checkPresetLvl(){
    if(m_wristSide){
      armPos = PresetConstants.lvl1to3ArmPosSide;
      switch (m_button_pressed) {
        // case 'a': elevatorPos = PresetConstants.lvl1ElevatorSide;
        //   break;
        case 'b': elevatorPos = PresetConstants.lvl2ElevatorSide;
          break;
        case 'x': elevatorPos = PresetConstants.lvl3ElevatorSide;
          break;
      }}
      else{
        switch (m_button_pressed) {

      case 'b': elevatorPos = PresetConstants.lvl2Elevator;
        break;

      case 'x': elevatorPos = PresetConstants.lvl3Elevator;
        break;

      case 'y': elevatorPos = PresetConstants.lvl4Elevator;
                armPos = PresetConstants.lvl4ArmPos;
                break;

      case'g': elevatorPos = PresetConstants.goundElevator;
                armPos = PresetConstants.groundArm;
                wristPos = PresetConstants.wristSidePos;
        break;
        }
      }

  }

}
