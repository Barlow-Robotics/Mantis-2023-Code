// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    /** Creates a new Arm. */

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings
    WPI_TalonFX extendMotor;
    WPI_TalonFX rotateMotorLeader; // 40:1 gearbox
    WPI_TalonFX rotateMotorFollower;

    double x = Constants.ArmConstants.RotateGearRatio;
    // boolean disableRotation = false;

    // public String armState;

    public enum Position {
        Resting, Bottom, Middle, Top, Floor, PlayerStation, Transition
    };

    public Position armState = Position.Resting;

    // BufferedTrajectoryPointStream bufferedStream = new
    // BufferedTrajectoryPointStream();

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.ArmExtendMotorID);
        rotateMotorLeader = new WPI_TalonFX(Constants.ArmConstants.ArmLeaderMotorID);
        rotateMotorFollower = new WPI_TalonFX(Constants.ArmConstants.ArmFollowMotorID);

        setExtendMotorConfig(extendMotor);
        setRotateMotorConfig(rotateMotorLeader);
        setRotateMotorConfig(rotateMotorFollower);

        rotateMotorFollower.follow(rotateMotorLeader);
        rotateMotorFollower.setInverted(InvertType.OpposeMaster);

        rotateMotorLeader.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);

        rotateMotorLeader.setSelectedSensorPosition(0);
        // extendMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // extendMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // extendMotor.configClearPositionOnLimitR(true, 0);

        // rotateMotorLeader.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // rotateMotorLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // rotateMotorLeader.configClearPositionOnLimitR(true, 0);
    }

    private void setExtendMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.setNeutralMode(NeutralMode.Brake);
        // motor.setNeutralMode(NeutralMode.Coast);
        motor.configClosedloopRamp(Constants.ArmConstants.LengthClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmConstants.LengthManualVoltageRampingConstant);
        motor.config_kF(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKF);
        motor.config_kP(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKP);
        motor.config_kI(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKI);
        motor.config_kD(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKD);
        motor.setInverted(TalonFXInvertType.Clockwise);
        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    private void setRotateMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configClosedloopRamp(Constants.ArmConstants.RotateClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmConstants.RotateManualVoltageRampingConstant);
        motor.config_kF(Constants.ArmConstants.RotatePID_id, Constants.ArmConstants.RotateKF);
        motor.config_kP(Constants.ArmConstants.RotatePID_id, Constants.ArmConstants.RotateKP);
        motor.config_kI(Constants.ArmConstants.RotatePID_id, Constants.ArmConstants.RotateKI);
        motor.config_kD(Constants.ArmConstants.RotatePID_id, Constants.ArmConstants.RotateKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
    }

    /* Rotate Motor */
    public void setAngle(double desiredAngle, double velocity, double accelerationTime) {
        // double currentPos = rotateMotorLeader.getSelectedSensorPosition();
        // double degrees = (currentPos - Constants.ArmConstants.kMeasuredPosHorizontal)
        // / Constants.ArmConstants.CountsPerArmDegree;
        // double radians = java.lang.Math.toRadians(degrees);
        // double cosineScalar = java.lang.Math.cos(radians);

        rotateMotorLeader
                .configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double currentAngle = getAngle();
        double ff = Math.sin(Math.toRadians(currentAngle)) * 0.75;

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
        // rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle); //ALH -
        // Before accounting for gravity
        // rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle,
        // DemandType.ArbitraryFeedForward,
        // Constants.maxGravityFF * cosineScalar); // ALH -
    }

    public double getAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmDegree;
        return result;
    }

    public boolean isAtMaxAngle() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinAngle() {
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    public void startRotating(double velocity) { // Velocity in degrees per second
        rotateMotorLeader.set(TalonFXControlMode.Velocity,
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.set(TalonFXControlMode.Velocity,
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
    }

    /* Extend Motor */
    public void setLength(double desiredLength, double velocity, double accelerationTime) { // 0.0in is when arm is
                                                                                            // fully retracted
        extendMotor.configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);
        extendMotor.configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double deltaDistance = desiredLength - getLength();
        double ff = Constants.ArmConstants.ffRetracted + ((deltaDistance / Constants.ArmConstants.ArmMaxLength)
                * (Constants.ArmConstants.ffExtracted - Constants.ArmConstants.ffRetracted));
        double feedForward = Math.sin(Math.toRadians(getAngle())) * ff;

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength, DemandType.ArbitraryFeedForward, feedForward);
    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean isAtMaxLength() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinLength() {
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    public void startExtending(double velocity) {
        extendMotor.set(TalonFXControlMode.Velocity, velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
    }

    /* Extend and Rotate */
    public void stopMoving() {
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    /* States */
    public void setState(Position returnValue) {
        armState = returnValue;
    }

    public Position getState() {
        return armState;
    }
}