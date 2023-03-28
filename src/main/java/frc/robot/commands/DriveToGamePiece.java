// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveToGamePiece extends CommandBase {

    Drive driveSub;
    Vision visionSub;
    Claw clawSub ;

    double targetDistance = 0.0;

    double startingLeftDistance;
    double startingRightDistance;

    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames = 0;
    private double adjustment;
    private boolean stopAtEnd ;

    public PIDController pid;

    public DriveToGamePiece(double speed, double distance, Drive d, Vision v, Claw c,  boolean stopAtEnd) {

        driveSub = d;
        visionSub = v;
        clawSub = c ;
        targetDistance = distance;
        leftVelocity = speed;
        rightVelocity = speed;
        stopAtEnd = stopAtEnd ;

        pid = new PIDController(
                Constants.DriveConstants.AutoAlignkP,
                Constants.DriveConstants.AutoAlignkI,
                Constants.DriveConstants.AutoAlignkD);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        // driveSub.resetOdometry(new Pose2d());
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
            double left = this.leftVelocity - adjustment;
            double right = this.rightVelocity + adjustment;

            driveSub.setSpeeds(left, right);
        } else {
            driveSub.setSpeeds(leftVelocity, rightVelocity);
        }

    }

    @Override
    public void end(boolean interrupted) {
        if ( stopAtEnd ) {
            driveSub.setSpeeds(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        double distanceTraveled = ((driveSub.getLeftDistance() - startingLeftDistance)
                + (driveSub.getRightDistance() - startingRightDistance)) / 2.0;
        // command is done if we've driven far enough or we've closed the claw.
        if (Math.abs(distanceTraveled) >= Math.abs(targetDistance) || !clawSub.isOpen()) {
            return true;
        }
        return false;
    }
}