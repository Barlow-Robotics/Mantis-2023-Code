// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase { // Extend, move to a certain place,
    /** Creates a new Arm. */

    WPI_TalonFX armTelescopeMotor;
    WPI_TalonFX armRotateMotor;
    public WPI_TalonFX leaderMotor;
    public WPI_TalonFX followMotor;


    public Arm() {
        armTelescopeMotor = new WPI_TalonFX(Constants.ArmConstants.armTelescopeMotorID);
        armRotateMotor = new WPI_TalonFX(Constants.ArmConstants.armRotateMotorID);
        leaderMotor = new WPI_TalonFX(Constants.ArmConstants.leaderMotorID);
        followMotor = new WPI_TalonFX(Constants.ArmConstants.followMotorID);
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
            leaderMotor.set(TalonFXControlMode.MotionMagic,desiredAngle);
            followMotor.set(TalonFXControlMode.MotionMagic, desiredAngle);
        }
    }

    

    public void setArmLength(double desiredLength) {
        // length in inches
        // 0.0in is when arm is fully retracted

        if (Math.abs(getArmAngle() - desiredLength) > ArmConstants.armAngleTolerance) {
            armRotateMotor.set(Constants.ArmConstants.armRotateSpeed);
        }
    }

    public double[] toPolarCoordiantes(double x, double y) {   // Convert from rectangular to polar coordinates
        double[] coordinates = new double[2];
        coordinates[0] = Math.sqrt((x*x)+(y*y));
        coordinates[1] = Math.atan(y/x);
        return coordinates;
    }

    public void toBottomRow() {
        // add polar coordinate path
    }

    public void toMiddleRow() {
        // add polar coordinate path
    }

    public void toTopRow() {
        // add polar coordinate path
    }

    public double getArmLength() {
        return 0.0;
    }
}
