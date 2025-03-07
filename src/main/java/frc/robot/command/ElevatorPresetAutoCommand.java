// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PresetConstants;

public class ElevatorPresetAutoCommand extends Command {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Intake m_intake;
    private double wristPos;
    private double armPos;
    private double elevatorPos;
    private char m_button_pressed;
    private boolean m_wristSide;
    private boolean canMoveElevator;
    private boolean canMoveArm;
    private boolean ranIntake;
    private boolean lvl4Auto;

    public ElevatorPresetAutoCommand(Elevator elevator, Arm arm, Intake intake, char button, boolean wristSide) {
        m_intake = intake;
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
        ranIntake = false;
        lvl4Auto = false;
        lvl4Auto &= m_elevator.getElevatorPos() > 4;
        lvl4Auto &= m_arm.getArmPos() > .42;

        if (armPos > 0.45) {
            canMoveArm &= MathUtil.isNear(0, m_elevator.getElevatorPos(), 0.1);
            if (!canMoveArm) {
                m_arm.setArmTarget(PresetConstants.armStartPos);
            }
        }
        if (armPos < 0.21) {
            canMoveArm &= m_elevator.getElevatorPos() > .63;
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

        if (m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean() && canMoveArm
                && canMoveElevator && lvl4Auto) {
            m_arm.setArmTarget(PresetConstants.armKickPos);
        } else if (m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean() && canMoveArm
                && canMoveElevator && !lvl4Auto) {
            m_intake.intakeEject();
            ranIntake = true;
        }

        // if (m_arm.getArmTarget() == PresetConstants.armKickPos && m_arm.getArmPos() < .32) {
        //     m_intake.intakeEject();
        //     ranIntake = true;
        // }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean() && canMoveArm
                && canMoveElevator && ranIntake;
    }

    /**
     * takes the wristSide boolean and button pressed to determine which preset
     * level to use
     */
    private void checkPresetLvl() {
        if (m_wristSide) {
            wristPos = PresetConstants.wristSidePos;
            armPos = PresetConstants.lvl2to3ArmPos;
            switch (m_button_pressed) {
                case 'a':
                    elevatorPos = PresetConstants.lvl1Elevator;
                    armPos = PresetConstants.lvl1ArmPos;
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
                    wristPos = PresetConstants.wristFlatPos;
                    break;
                case 'g':
                    elevatorPos = PresetConstants.goundElevatorPos;
                    armPos = PresetConstants.groundArmPos;
                    wristPos = PresetConstants.wristSidePosNeg;
                    break;
                case 'q':
                    elevatorPos = PresetConstants.elevatorAlgaeGroundPos;
                    armPos = PresetConstants.armAlgaeGroundPos;
                    break;
            }
        } else {
            wristPos = PresetConstants.wristFlatPos;
            armPos = PresetConstants.lvl2to3ArmPos;
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
                // elevatorPos = PresetConstants.startElevator;
                // armPos = PresetConstants.groundArm;
                // break;
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
