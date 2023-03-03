// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

    private void setExtendMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configClosedloopRamp(Constants.ArmConstants.LengthClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmConstants.LengthManualVoltageRampingConstant);
        motor.config_kF(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKF);
        motor.config_kP(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKP);
        motor.config_kI(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKI);
        motor.config_kD(Constants.ArmConstants.LengthPID_id, Constants.ArmConstants.LengthKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    private void setRotateMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
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
        // if (getAngle() <= Constants.ArmConstants.ArmMinAngle || getAngle() >=
        // Constants.ArmConstants.ArmMaxAngle) {
        // disableRotation = true;
        // }
        // if (getAngle() <= 25) {
        // setLength(0, Constants.ArmConstants.armExtendSpeed, 2);
        // }
    }

    public double getAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmDegree;
        return result;
    }

    public void setAngle(double desiredAngle, double velocity, double accelerationTime) {
        rotateMotorLeader
                .configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);
        rotateMotorLeader
                .configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public boolean isAtMaxAngle() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1 * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec;
    }

    public boolean isAtMinAngle() {
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1 * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec;
    }

    public void startRotatingAtVelocty(double velocity) { // Velocity in degrees per second
        rotateMotorLeader.set(TalonFXControlMode.Velocity,
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.set(TalonFXControlMode.Velocity,
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
    }

    public void setLength(double desiredLength, double velocity, double accelerationTime) {
        // inches
        // 0.0in is when arm is fully retracted

        // if (Math.abs(getArmLength() - desiredLength) >
        // Constants.ArmConstants.armLengthTolerance) {
        // extendMotor.set(Constants.ArmConstants.armExtendSpeed);
        // }

        extendMotor.configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);
        extendMotor.configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength);
    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean isAtMaxExtension() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinExtension() {
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    public void startExtendingAtVelocty(double velocity) {
        extendMotor.set(TalonFXControlMode.Velocity, velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
    }

    public void stopMoving() { // Make neutral mode for extaend and rotate motors to brake
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void setState(Position returnValue) {
        armState = returnValue;
    }

    public Position getState() {
        return armState;
    }

    // public void startProfiles() {
    // rotateMotorLeader.startMotionProfile(null, 0, ControlMode.MotionProfile); //
    // Rotate profile
    // extendMotor.startMotionProfile(null, 0, ControlMode.MotionProfile); // Extend
    // profile
    // }

    // public boolean isProfileComplete() {
    // return extendMotor.isMotionProfileFinished();
    // }
}