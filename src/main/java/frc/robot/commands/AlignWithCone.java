// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class AlignWithCone extends CommandBase {
  /** Creates a new AlignWithCone. */

  private PIDController pid = new PIDController(0.01, 0, 0);
    private Drive driveSub;
    private Vision visionSub;

    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames = 0;
    private double adjustment;
    
  public AlignWithCone(Vision v, Drive d) {
    driveSub = d;
    visionSub = v;
    addRequirements(driveSub, visionSub);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
    pid.reset();
    missedFrames = 0;
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    if (visionSub.coneIsVisible() && (visionSub.bbConeWidth() > 50 || visionSub.bbConeHeight() > 85)) {
        error = visionSub.coneDistanceFromCenter();
        adjustment = pid.calculate(error);
        adjustment = Math.signum(adjustment)
                * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
        leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
        rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;

        driveSub.setSpeeds(leftVelocity, rightVelocity);
    } else {
        missedFrames++;
    }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    driveSub.setSpeeds(0, 0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    if (missedFrames > 10) {
        return true;
    } else {
        return false;
    }
}
}
