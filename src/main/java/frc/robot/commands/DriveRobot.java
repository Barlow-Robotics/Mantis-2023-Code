// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobot extends CommandBase {
  /** Creates a new Drive. */

  Drive driveSub;
  Claw clawSub;
  Vision visionSub;
  RobotContainer robotCont;

  private boolean lastAutoSteer = false;
  private float yawMultiplier = 1.0f;

  public DriveRobot(Drive d, Claw c, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.

    driveSub = d;
    clawSub = c;
    visionSub = v;
    addRequirements(driveSub, clawSub, visionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     boolean autoSteer = robotCont.autoAlignButton.getAsBoolean();

    double x = robotCont.driverController.getRawAxis(robotCont.xAxis);
    if (Math.abs(x) < 0.01) {
      x = 0.0;
    }
    double yaw = -robotCont.driverController.getRawAxis(robotCont.yawAxis);
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
        robotCont.pid.reset();
      }
      yaw = -robotCont.pid.calculate(visionSub.gamePieceDistanceFromCenter());
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
