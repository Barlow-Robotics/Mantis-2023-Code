// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
   
    private Drive driveSub;

    public AutoBalance(Drive d) {
        driveSub = d;
        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double error = driveSub.getPitch();

        if (Math.abs(error) >= Constants.DriveConstants.BalanceTolerance) {
            driveSub.setSpeeds(
                Constants.DriveConstants.AutoBalanceSpeed * -Math.signum(error), 
                Constants.DriveConstants.AutoBalanceSpeed * -Math.signum(error));
        } else {
            driveSub.setSpeeds(0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}