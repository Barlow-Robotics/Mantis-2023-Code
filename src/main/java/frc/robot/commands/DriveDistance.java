// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveDistance extends CommandBase {

    Drive driveSub;

    double targetDistance; //meters?
    double speed; //meters per second?

    double startingLeftDistance;
    double startingRightDistance;

    public DriveDistance(Drive d, double dist, double s) {
        driveSub = d;
        targetDistance = dist;
        speed = s;
        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        driveSub.resetOdometry(new Pose2d());
        startingLeftDistance = driveSub.getLeftDistance();
        startingRightDistance = driveSub.getRightDistance();
    }

    @Override
    public void execute() {
        driveSub.setSpeeds(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.setSpeeds(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        double distanceTraveled = ((driveSub.getLeftDistance() - startingLeftDistance)
                + (driveSub.getRightDistance() - startingRightDistance)) / 2.0;
        if (Math.abs(distanceTraveled) >= Math.abs(targetDistance)) {
            return true;
        }
        return false;
    }
}