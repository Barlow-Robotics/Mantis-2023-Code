// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveToGamePiece extends CommandBase {

    Drive driveSub;
    Vision visionSub;

    double targetDistance; //meters?

    double startingLeftDistance;
    double startingRightDistance;

    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames = 0;
    private double adjustment;

    public PIDController pid;

    public DriveToGamePiece(Drive d, Vision v, double dist) {
        driveSub = d;
        visionSub = v;
        targetDistance = dist;

        pid = new PIDController(
                Constants.DriveConstants.AutoAlignkP,
                Constants.DriveConstants.AutoAlignkI,
                Constants.DriveConstants.AutoAlignkD);

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

        if (visionSub.gamePieceIsVisible()) {
            error = visionSub.gamePieceDistanceFromCenter();
            adjustment = pid.calculate(error);
            adjustment = Math.signum(adjustment)
                    * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
            leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
            rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;

            driveSub.setSpeeds(leftVelocity, rightVelocity);
        } else {
            missedFrames++;
        }

        driveSub.setSpeeds(Constants.AutoConstants.DriveToGamePieceSpeed, Constants.AutoConstants.DriveToGamePieceSpeed);
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