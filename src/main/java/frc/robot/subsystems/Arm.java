// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase { // Extend, move to a certain place,
    /** Creates a new Arm. */

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings
    WPI_TalonFX extendMotor;
    WPI_TalonFX rotateMotorLeader; // 40:1 gearbox
    WPI_TalonFX rotateMotorFollower;

    double x = Constants.ArmConstants.rotateGearRatio;

    int state = 0;

    // BufferedTrajectoryPointStream bufferedStream = new BufferedTrajectoryPointStream();

    // Need to change these to ARM constants, not Drive
    private void setMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.DriveConstants.PID_id, Constants.DriveConstants.kF);
        motor.config_kP(Constants.DriveConstants.PID_id, Constants.DriveConstants.kP);
        motor.config_kI(Constants.DriveConstants.PID_id, Constants.DriveConstants.kI);
        motor.config_kD(Constants.DriveConstants.PID_id, Constants.DriveConstants.kD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.armExtendMotorID);
        rotateMotorLeader = new WPI_TalonFX(Constants.ArmConstants.armLeaderMotorID);
        rotateMotorFollower = new WPI_TalonFX(Constants.ArmConstants.armFollowMotorID);

        setMotorConfig(extendMotor);
        setMotorConfig(rotateMotorLeader);
        setMotorConfig(rotateMotorFollower);

        rotateMotorFollower.follow(rotateMotorLeader);

        rotateMotorLeader.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);

        extendMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extendMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        // wpk, should we also use the TalonFX built in "soft" limit switches?

        // wpk, we can automatically reset the encoder when the retracted (reverse)
        // limit is reached. Do we want to do this?
        // If so, it would look like this:
        extendMotor.configClearPositionOnLimitR(true, 0);

        rotateMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
        rotateMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);

        rotateMotorLeader.configClearPositionOnLimitR(true, 0);

    }

    @Override
    public void periodic() {}

    public double getAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmDegree;
        return result;
    }

    public void setAngle(double desiredAngle, double cruiseVelocity, double acceleration) {
        rotateMotorLeader.configMotionCruiseVelocity(cruiseVelocity);
        rotateMotorLeader.configMotionAcceleration(acceleration);

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public boolean IsAtMaxAngle() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean IsAtMinAngle() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    /* @param velocity in degrees per second */
    public void StartRotatingAtVelocty(double velocity) {
        /* @param velocity in degrees per second */
        rotateMotorLeader.set(TalonFXControlMode.Velocity, velocity);
    }

    public void setLength(double desiredLength, double velocity, double acceleration) {
        // inches
        // 0.0in is when arm is fully retracted

        // if (Math.abs(getArmLength() - desiredLength) >
        // Constants.ArmConstants.armLengthTolerance) {
        // extendMotor.set(Constants.ArmConstants.armExtendSpeed);
        // }

        extendMotor.configMotionCruiseVelocity(velocity);
        extendMotor.configMotionAcceleration(acceleration);

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength);
    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean IsAtMaxExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean IsAtMinExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    public void StartExtendingAtVelocty(double velocity) {
        extendMotor.set(TalonFXControlMode.Velocity, velocity);
    }

    public void StopMoving() {
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void startProfiles() {
        rotateMotorLeader.startMotionProfile(null, 0, ControlMode.MotionProfile); // Rotate profile
        extendMotor.startMotionProfile(null, 0, ControlMode.MotionProfile); // Extend profile
    }

    public boolean isProfileComplete() {
        return extendMotor.isMotionProfileFinished();
    }
}