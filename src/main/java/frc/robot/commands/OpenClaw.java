// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {

    Claw clawSub;

    public OpenClaw(Claw c) {

        clawSub = c;
        addRequirements(clawSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        clawSub.open();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}