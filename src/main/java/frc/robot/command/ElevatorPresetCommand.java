// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PresetConstants;

public class ElevatorPresetCommand extends Command {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private double wristPos;
    private double armPos;
    private double elevatorPos;
    private char m_button_pressed;
    private boolean m_wristSide;
    private boolean canMoveElevator;
    private boolean canMoveArm;

    public ElevatorPresetCommand(Elevator elevator, Arm arm, char button, boolean wristSide) {
        m_elevator = elevator; // saves a local reference to the elevator subsystem
        m_arm = arm; // saves a local reference to the arm subsystem
        m_button_pressed = button; // saves a local reference to what button was pressed
        m_wristSide = wristSide; // saves a local reference if whether the wirst needs to be on its side or not
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator, arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_elevator.resetPidError();
        // m_arm.resetPidError();
        checkPresetLvl();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        canMoveElevator = m_arm.getArmPos() < 0.45;
        canMoveArm = true;
        if (armPos > 0.45) {
            canMoveArm &= MathUtil.isNear(0, m_elevator.getElevatorPos(), 0.1);
            if (!canMoveArm) {
                m_arm.setArmTarget(PresetConstants.armStartPos);
            }
        }
        if (armPos < 0.24) {
            canMoveArm &= m_elevator.getElevatorPos() > .94;
            if (!canMoveArm) {
                m_arm.setArmTarget(PresetConstants.armStartPos);
            }

        }
        if (canMoveElevator) {
            m_elevator.targetElevatorPosition(elevatorPos);
        }

        m_elevator.updateElevatorPID();

        if (canMoveArm) {
            m_arm.setArmMechTarget(wristPos, armPos);
        }

        m_arm.setArmSpeed();
        m_arm.setWristSpeed();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean() && canMoveArm
                && canMoveElevator;
    }

    /**
     * takes the wristSide boolean and button pressed to determine which preset
     * level to use
     */
    private void checkPresetLvl() {
        if (m_wristSide) {
            wristPos = PresetConstants.wristSidePos;
            armPos = PresetConstants.lvl1to3ArmPos;
            switch (m_button_pressed) {
                case 'a':
                    elevatorPos = PresetConstants.lvl1Elevator;
                    break;
                case 'b':
                    elevatorPos = PresetConstants.lvl2Elevator;
                    break;
                case 'x':
                    elevatorPos = PresetConstants.lvl3Elevator;
                    break;
                case 'h':
                    elevatorPos = PresetConstants.startElevator;
                    armPos = PresetConstants.humanIntakeArmPos;
                    break;
                case 'g':
                    elevatorPos = PresetConstants.goundElevatorPos;
                    armPos = PresetConstants.groundArmPos;
                    wristPos = PresetConstants.wristSidePosNeg;
                    break;
            }
        } else {
            wristPos = PresetConstants.wristFlatPos;
            armPos = PresetConstants.lvl1to3ArmPos;
            switch (m_button_pressed) {

                case 'b':
                    elevatorPos = PresetConstants.lvl2Elevator;
                    break;

                case 'x':
                    elevatorPos = PresetConstants.lvl3Elevator;
                    break;

                case 'y':
                    elevatorPos = PresetConstants.lvl4Elevator;
                    armPos = PresetConstants.lvl4ArmPos;
                    break;

                // case 'g':
                //     elevatorPos = PresetConstants.startElevator;
                //     armPos = PresetConstants.groundArm;
                //     break;
                case 't':
                    elevatorPos = PresetConstants.startElevator;
                    armPos = PresetConstants.travelArm;
                    break;
                case 'h':
                    elevatorPos = PresetConstants.humanElevatorPos;
                    armPos = PresetConstants.directHumanArmPos;
                    wristPos = PresetConstants.wristSidePos;
                    break;
                case 's':
                    elevatorPos = PresetConstants.startElevator;
                    armPos = PresetConstants.armStartPos;
                    break;
            }
        }
    }
}
