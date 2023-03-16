// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
    /** Creates a new Engage. */
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
        // Does the navX need to be zeroed/reset/calibrated?
        if (Math.abs(error) >= Constants.DriveConstants.BalanceTolerance) {
            driveSub.setSpeeds(Constants.DriveConstants.AutoBalanceSpeed * error, Constants.DriveConstants.AutoBalanceSpeed * error);
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
