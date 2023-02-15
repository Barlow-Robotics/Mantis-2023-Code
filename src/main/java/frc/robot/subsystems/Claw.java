// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    /** Creates a new Claw. */

    WPI_TalonFX clawMotor; // For adjusting angle
    // Need to add piston to open claw (look at Tarzan code)

    Arm armSub;

    public Claw(Arm a) { // add arm to constructors
        armSub = a;
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.clawMotorID); // needs config 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // 0.0 is perpendicular to arm bar
        setClawAngle(90 - armSub.getAngle()); // Probably doesnt work (?)
    }

    public double getClawAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.CountsPerClawDegree;
        return result;
    }

    // wpk - why not set desired positon using motion magic or closed loop position control.
    public void setClawAngle(double desiredAngle) {
        // if (Math.abs(getClawAngle() - desiredAngle) > ClawConstants.ClawAngleTolerance) {
        //     clawMotor.set(Constants.ClawConstants.clawSpeed);
        // }
        double setAngle = desiredAngle * ClawConstants.CountsPerClawDegree;
        clawMotor.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public void openClaw() {
        // Need to make this, need piston
    }

    public void closeClaw() {
        // Need to make this, need piston
    }

    public boolean clawIsOpen() {
        return true; // Need to make this
    }
}
