// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/** An command that uses an subsystem. */
public class FlipWrist90Command extends Command {
    private final Arm m_arm;

    /**
     * Creates a new Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FlipWrist90Command(Arm arm) {
        m_arm = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
        getInterruptionBehavior();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        checkWristPos();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.setWristSpeed();
        m_arm.setArmSpeed();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.setWristTarget(m_arm.getWristPosition());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_arm.armMechAtTarget.getAsBoolean();
    }

    private void checkWristPos() {
        if (MathUtil.isNear(PresetConstants.wristSidePos, m_arm.getWristPosition(),
                PresetConstants.wristTolerance)) {
            m_arm.setWristTarget(PresetConstants.wristFlatPos);
        }

        else if (MathUtil.isNear(PresetConstants.wristFlatPos, m_arm.getWristPosition(),
                PresetConstants.wristTolerance)) {
            m_arm.setWristTarget(PresetConstants.wristSidePos);
        }

        else if (MathUtil.isNear(PresetConstants.wristSidePosNeg, m_arm.getWristPosition(),
                PresetConstants.wristTolerance)) {
            m_arm.setWristTarget(PresetConstants.wristFlatPosNeg);
        }

        else if (MathUtil.isNear(PresetConstants.wristFlatPosNeg, m_arm.getWristPosition(),
                PresetConstants.wristTolerance)) {
            m_arm.setWristTarget(PresetConstants.wristSidePosNeg);
        } else {
            m_arm.setWristTarget(PresetConstants.wristFlatPos);
        }
    }
}
