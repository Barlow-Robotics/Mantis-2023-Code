// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class EngageChargingStation extends CommandBase {
    /** Creates a new Engage. */
    public final AHRS navX = new AHRS();

    private Drive driveSub;

    public EngageChargingStation(Drive d) {
        driveSub = d;
        addRequirements(driveSub);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double error = 0 - navX.getPitch();

        if (Math.abs(navX.getAngle()) >= Constants.DriveConstants.balanceTolerance) {
            driveSub.setSpeeds(Constants.DriveConstants.kP * error, Constants.DriveConstants.kP * error);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(navX.getAngle()) <= Constants.DriveConstants.balanceTolerance);
    }
}
