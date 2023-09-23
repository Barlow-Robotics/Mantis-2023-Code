// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class CalibrateArmExtention extends CommandBase {

    private Arm armSub;

    public CalibrateArmExtention(Arm a) {
        armSub = a;
        addRequirements(armSub);
    }

    @Override
    public void initialize() {
        armSub.startExtending(Constants.ArmConstants.ExtentionCalibrationVelocity);
    }

    @Override
    public void execute() {
        armSub.startExtending(Constants.ArmConstants.ExtentionCalibrationVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        armSub.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return armSub.isAtMinLength();
    }
}