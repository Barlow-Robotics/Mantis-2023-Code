// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MoveToRestingPosition extends CommandBase {
    /** Creates a new MoveToRestingPosition. */

    Arm armSub;
    Claw clawSub;

    double armLength;
    double armAngle;
    double clawAngle;

    public MoveToRestingPosition(Arm a, Claw c) {
        armSub = a;
        clawSub = c;

        addRequirements(armSub, clawSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        clawSub.openClaw();
        // armSub.setArmAngle(Constants.ArmConstants.restingArmAngle);
        armSub.setArmLength(Constants.ArmConstants.restingArmLength);
        // clawSub.setClawAngle(Constants.ClawConstants.restingClawAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(armSub.getArmAngle()
                - Constants.ArmConstants.restingArmAngle) < Constants.ArmConstants.armAngleTolerance);
    }

}
