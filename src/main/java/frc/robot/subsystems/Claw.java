// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    /** Creates a new Claw. */

    WPI_TalonFX clawMotor; // For adjusting angle
    // Need to add piston to open claw

    Arm armSub;

    public Claw(Arm a) {
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.clawMotorID);
        armSub = a;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // 0.0 is perpendicular to arm bar
        setClawAngle(-armSub.getArmAngle()); // Probably doesnt work (?)
    }

    public double getClawAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.UnitsPerClawDegree;
        return result;
    }

    public void setClawAngle(double desiredAngle) {
        if (Math.abs(getClawAngle() - desiredAngle) > ClawConstants.ClawAngleTolerance) {
            clawMotor.set(Constants.ClawConstants.clawSpeed);
        }
    }

    public void openClaw() {
        // Need to make this
    }

    public void closeClaw() {
        // Need to make this
    }

    public boolean clawIsOpen() {
        return true; // Need to make this
    }
}