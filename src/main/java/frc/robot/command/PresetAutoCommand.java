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

public class PresetAutoCommand extends Command {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Intake m_intake;
    private double wristPos;
    private double armPos;
    private double elevatorPos;
    private int presetID;
    private boolean m_wristSide;
    private boolean canMoveArm;

    public PresetAutoCommand(Elevator elevator, Arm arm, Intake intake, int preset) {
        m_intake = intake;
        m_elevator = elevator; // saves a local reference to the elevator subsystem
        m_arm = arm; // saves a local reference to the arm subsystem
        // Use addRequirements() here to declare subsystem dependencies.
        presetID = preset;
        addRequirements(elevator, arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        presetSelector();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        canMoveArm = true;

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

        if (canMoveArm) {
            m_arm.setArmTarget(armPos);
        }
        m_arm.setArmSpeed();

        if (m_arm.getArmPos() < .3) {
            m_intake.intakeEject();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_elevator.elevatorAtTarget.getAsBoolean() && m_arm.armMechAtTarget.getAsBoolean() && canMoveArm;
    }

    private void presetSelector() {
        switch (presetID) {
            case 0:
                armPos = .24;
                break;
            case 1:
                armPos = PresetConstants.humanIntakeArmPos;
                break;
        }
    }

}
