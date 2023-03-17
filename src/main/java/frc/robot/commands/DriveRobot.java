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
import frc.robot.RobotContainer;
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

    Trigger autoAlignButton;
    Joystick driverController;
    int xAxis;
    int yawAxis;

    public PIDController pid;

    public DriveRobot(Drive d, Claw c, Vision v, Trigger autoAlignButton, Joystick driverController, int xAxis,
            int yawAxis) {
        // Use addRequirements() here to declare subsystem dependencies.

        driveSub = d;
        clawSub = c;
        visionSub = v;
        this.autoAlignButton = autoAlignButton;
        this.driverController = driverController;
        this.xAxis = xAxis;
        this.yawAxis = yawAxis;

        pid = new PIDController(
            Constants.DriveConstants.autoAlignkP, Constants.DriveConstants.autoAlignkI, Constants.DriveConstants.autoAlignkD);
        
        addRequirements(driveSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean autoSteer = autoAlignButton.getAsBoolean();

        SmartDashboard.putBoolean("Auto Steer", autoSteer) ;

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

        if (!autoSteer || !clawSub.isOpen()) {
            yaw = -turn;

            // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);
            yawMultiplier = 0.5f;

            double yawSign = 1.0;
            if (yaw < 0.0) {
                yawSign = -1.0;
            }
            yaw = yawSign * (yaw * yaw)
                    * yawMultiplier;
            if (Math.abs(yaw) < 0.02f) {
                yaw = 0.0f;
            }
            lastAutoSteer = false;
        } else {
            if (!lastAutoSteer) {
                pid.reset();
            }
            yaw = pid.calculate(visionSub.gamePieceDistanceFromCenter());
            lastAutoSteer = true;
        }
        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

        driveSub.drive(-speed, yaw * 0.8, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
