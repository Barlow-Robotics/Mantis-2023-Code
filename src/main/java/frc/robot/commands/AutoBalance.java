// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {

    private Drive driveSub;
    double ff = 0.0;
    double previousError;

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
        if (Math.abs(driveSub.getPitch()) < 2.5) {
            error = 0;
        } else if (Math.abs(driveSub.getPitch()) > 14) {
            error = 14 * Math.signum(error);
        }

        driveSub.setSpeedsWithFF(DriveConstants.AutoBalanceSpeed * -error,
                                 DriveConstants.AutoBalanceSpeed * -error,
                                 ff, ff);

        // double error = driveSub.getPitch();

        // if (Math.abs(error) >= DriveConstants.BalanceMaxAngle) {
        // driveSub.setSpeedsWithFF(
        // DriveConstants.AutoBalanceSpeed * -Math.signum(error),
        // DriveConstants.AutoBalanceSpeed * -Math.signum(error),
        // ff, ff);
        // // -14 degrees = 0.15 meters per sec
        // } else if (Math.abs(error) > DriveConstants.BalanceMinAngle &&
        // Math.abs(error) < DriveConstants.BalanceMaxAngle) {
        // driveSub.setSpeedsWithFF(
        // DriveConstants.AutoBalanceKP * -error,
        // DriveConstants.AutoBalanceKP * -error,
        // ff, ff);
        // // -13.999... degrees = 0.20999... meters per sec
        // } else {
        // driveSub.setSpeeds(0.0, 0.0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getFF() {
        return this.ff;
    }

    public void setFF(double value) {
        ff = value;
    }

    // public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty("Feed Forward", this::getFF, this::setFF);
    // }

}