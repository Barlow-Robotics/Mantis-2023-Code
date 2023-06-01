// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CalibrateClaw extends CommandBase {

    Claw clawSub;

    public CalibrateClaw(Claw clawSub) {
        this.clawSub = clawSub;
        addRequirements(clawSub);
    }

    @Override
    public void initialize() {
        clawSub.startMoving();
    }

    @Override
    public void execute() {
        clawSub.startMoving();
    }

    @Override
    public void end(boolean interrupted) {
        clawSub.resetEncoders();
        clawSub.stopMoving();
    }

    @Override
    public boolean isFinished() {
        System.out.println("****** CLAW SUPPLY CURRENT IS " + clawSub.getSupplyCurrent() + " ******");
        return clawSub.getSupplyCurrent() >= 1.7;
    }
}
