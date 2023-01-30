// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm extends SubsystemBase { // Extend, move to a certain place,
    /** Creates a new Arm. */

    WPI_TalonFX armTelescopeMotor;
    WPI_TalonFX armRotateMotor;

    public Arm() {
        armTelescopeMotor = new WPI_TalonFX(Constants.ArmConstants.armTelescopeMotorID);
        armRotateMotor = new WPI_TalonFX(Constants.ArmConstants.armRotateMotorID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    public double getArmAngle() {
        double result = armRotateMotor.getSelectedSensorPosition() / Constants.ArmConstants.UnitsPerArmDegree;
        return result;
    }

    public void setArmAngle(double desiredAngle) {
        // angle in degrees
        // EP need to determine what 0.0 degrees entails

        if (Math.abs(getArmAngle() - desiredAngle) > ArmConstants.armAngleTolerance) {
            armRotateMotor.set(Constants.ArmConstants.armRotateSpeed);
        }
    }

    public void setArmLength(double desiredLength) {
        // length in inches
        // 0.0in is when arm is fully retracted
    }

    // public double[] polarCoordiantes(double x, double y) {   // Convert from rectangular to polar coordinates
    //     double[] coordinates = new double[2];
    //     coordinates[0] = Math.sqrt((x*x)+(y*y));
    //     coordinates[1] = Math.atan(y/x);
    //     return coordinates;
    // }

    public double getArmLength() {
        return 0.0;
    }
}