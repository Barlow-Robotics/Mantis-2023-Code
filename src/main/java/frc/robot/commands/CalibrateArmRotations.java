// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateArmRotations extends CommandBase {

    private Arm armSub;

    public CalibrateArmRotations(Arm a) {
        armSub = a;
        addRequirements(armSub);
    }

    @Override
    public void initialize() {
        armSub.startRotating(Constants.ArmConstants.RotationCalibrationVelocity);
    }

    @Override
    public void execute() {
        // armSub.StartRotatingAtVelocty(Constants.ArmConstants.RotationCalibrationVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        armSub.startRotating(0);
    }

    @Override
    public boolean isFinished() {
        return armSub.isAtMinAngle();
    }
}