// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobot extends CommandBase {
    /** Creates a new Drive. */

    Drive driveSub;
    Claw clawSub;
    Vision visionSub;

    private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;
    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames = 0;
    private double adjustment;

    Trigger autoAlignButton;
    Trigger toggleTargetButton;
    Joystick driverController;
    int xAxis;
    int yawAxis;

    public PIDController pid;

    public String selectedTarget = "None";

    public DriveRobot(
            Drive d, Claw c, Vision v, Trigger autoAlignButton, Trigger toggleTargetButton, Joystick driverController,
            int xAxis, int yawAxis) {

        driveSub = d;
        clawSub = c;
        visionSub = v;
        this.autoAlignButton = autoAlignButton;
        this.toggleTargetButton = toggleTargetButton;
        this.driverController = driverController;
        this.xAxis = xAxis;
        this.yawAxis = yawAxis;

        pid = new PIDController(
                Constants.DriveConstants.autoAlignkP,
                Constants.DriveConstants.autoAlignkI,
                Constants.DriveConstants.autoAlignkD);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        pid.reset();
        missedFrames = 0;
    }

    @Override
    public void execute() {
        boolean autoAlignEnabled = autoAlignButton.getAsBoolean();
        boolean toggleTarget = toggleTargetButton.getAsBoolean();

        SmartDashboard.putBoolean("Auto Align Enabled", autoAlignEnabled);
        SmartDashboard.putString("Auto Align Target", selectedTarget);

        double x = driverController.getRawAxis(xAxis);
        if (Math.abs(x) < 0.01) {
            x = 0.0;
        }
        double yaw = -driverController.getRawAxis(yawAxis);
        if (Math.abs(yaw) < 0.01) {
            yaw = 0.0;
        }
        double speed = -x;

        // // If we're going forward, use "full" speed
        // if (speed > 0.0) {
        // speed = speed * 0.5;
        // } else {
        // // we're going backward, so use slower speed
        // speed = speed * 0.75;
        // }
        double turn = -yaw;

        if (!autoAlignEnabled || !clawSub.isOpen()) {
            yaw = -turn;

            // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);
            yawMultiplier = 0.5f;
            double yawSign = 1.0;

            if (yaw < 0.0) {
                yawSign = -1.0;
            }

            yaw = yawSign * (yaw * yaw) * yawMultiplier;

            if (Math.abs(yaw) < 0.02f) {
                yaw = 0.0f;
            }

            lastAutoSteer = false;
        } else {
            if (!lastAutoSteer) {
                pid.reset();
            }

            if (toggleTarget == true) { /* switch indicates game piece with switch value of 1 (maybe or 0?) */

                selectedTarget = "Game Piece";

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
            } else { /* switch indicates april tag with switch value of -1 */

                selectedTarget = "April Tag";

                if (visionSub.aprilTagIsVisible()) {
                    error = visionSub.aprilTagDistanceFromCenter();
                    adjustment = pid.calculate(error);
                    adjustment = Math.signum(adjustment)
                            * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
                    leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
                    rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;

                    driveSub.setSpeeds(leftVelocity, rightVelocity);
                } else {
                    missedFrames++;
                }
                yaw = pid.calculate(visionSub.gamePieceDistanceFromCenter());
                lastAutoSteer = true;
            }
        }

        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

        driveSub.drive(-speed, yaw * 0.8, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}