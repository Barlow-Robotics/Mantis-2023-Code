// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotionProfile;

public class Arm extends SubsystemBase { // Extend, move to a certain place,
    /** Creates a new Arm. */

    WPI_TalonFX extendMotor;
    WPI_TalonFX rotateMotorLeader;
    WPI_TalonFX rotateMotorFollower;

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings

    Joystick opp = new Joystick(0);

    int state = 0;

    WPI_TalonFX master = new WPI_TalonFX(1, "rio");

    BufferedTrajectoryPointStream _bufferedStream = new BufferedTrajectoryPointStream();

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.armTelescopeMotorID);
        rotateMotorLeader = new WPI_TalonFX(Constants.ArmConstants.leaderMotorID);
        rotateMotorFollower = new WPI_TalonFX(Constants.ArmConstants.followMotorID);

        rotateMotorFollower.follow(rotateMotorLeader);
    }

    @Override
    public void periodic() {
    }

    public double getArmAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.UnitsPerArmDegree;
        return result;
    }

    public void setArmLength(double desiredLength) {
        // length in inches
        // 0.0in is when arm is fully retracted

        if (Math.abs(getArmAngle() - desiredLength) > Constants.ArmConstants.armAngleTolerance) {
            rotateMotorLeader.set(Constants.ArmConstants.armRotateSpeed);
        }
    }

    public double[] toPolarCoordiantes(double x, double y) { // Convert from rectangular to polar coordinates
        double[] coordinates = new double[2];
        coordinates[0] = Math.sqrt((x * x) + (y * y));
        coordinates[1] = Math.atan(y / x);
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

    public void startProflies() {
        // Rotate profile
        rotateMotorLeader.startMotionProfile(null, 0, null);
        // Extend profile
        extendMotor.startMotionProfile(null, 0, null);
    }

    public boolean isProfileComplete() {
        return false;
    }

}