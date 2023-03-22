// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArmManual extends CommandBase {

    private Arm armSub;

    Joystick operatorController;

    private int controllerAngleID;
    private int controllerExtensionID;

    public MoveArmManual(Arm a, Joystick opController, int angleID, int extensionID) {
        armSub = a;
        operatorController = opController;
        controllerAngleID = angleID;
        controllerExtensionID = extensionID;
        addRequirements(armSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /* Angle */
        double currentAngle = armSub.getAngle();
        double desiredAngle = 
                currentAngle + (operatorController.getRawAxis(controllerAngleID) * ArmConstants.AngleMultiplier);

        if (desiredAngle > ArmConstants.ArmMaxAngle) {
            desiredAngle = ArmConstants.ArmMaxAngle;
        } else if (desiredAngle < ArmConstants.ArmMinAngle) {
            desiredAngle = ArmConstants.ArmMinAngle;
        }

        armSub.setAngle(desiredAngle, ArmConstants.AngleVel, ArmConstants.AngleAcceleration);

        /* Extension */
        double currentLength = armSub.getLength();
        double desiredLength = currentLength + operatorController.getRawAxis(controllerExtensionID)
                * Constants.ArmConstants.LengthMultiplier;

        if (desiredLength > ArmConstants.ArmMaxLength) {
            desiredLength = ArmConstants.ArmMaxLength;
        } else if (desiredLength < ArmConstants.ArmMinLength) {
            desiredLength = ArmConstants.ArmMinLength;
        }

        armSub.setLength(desiredLength, ArmConstants.ExtendVel, ArmConstants.ExtendAccel);

        if (desiredLength * Math.cos(desiredAngle) <= 0) {
            desiredLength = currentLength;
        }

        armSub.setAngle(desiredAngle, Constants.ArmConstants.AngleVel, ArmConstants.AngleAcceleration);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}