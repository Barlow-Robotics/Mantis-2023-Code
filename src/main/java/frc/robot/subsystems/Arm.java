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

    double x = Constants.ArmConstants.rotateGearRatio;

    int state = 0;

    // BufferedTrajectoryPointStream bufferedStream = new
    // BufferedTrajectoryPointStream();

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.armExtendMotorID);
        rotateMotorLeader = new WPI_TalonFX(Constants.ArmConstants.armLeaderMotorID);
        rotateMotorFollower = new WPI_TalonFX(Constants.ArmConstants.armFollowMotorID);

        setExtendMotorConfig(extendMotor);
        setRotateMotorConfig(rotateMotorLeader);
        setRotateMotorConfig(rotateMotorFollower);

        rotateMotorFollower.follow(rotateMotorLeader);

        rotateMotorLeader.configMotionSCurveStrength(Constants.ArmConstants.accelerationSmoothing);

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
        motor.configClosedloopRamp(Constants.ArmConstants.lengthClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmConstants.lengthManualVoltageRampingConstant);
        motor.config_kF(Constants.ArmConstants.lengthPID_id, Constants.ArmConstants.lengthKF);
        motor.config_kP(Constants.ArmConstants.lengthPID_id, Constants.ArmConstants.lengthKP);
        motor.config_kI(Constants.ArmConstants.lengthPID_id, Constants.ArmConstants.lengthKI);
        motor.config_kD(Constants.ArmConstants.lengthPID_id, Constants.ArmConstants.lengthKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }
    
    private void setRotateMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configClosedloopRamp(Constants.ArmConstants.rotateClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmConstants.rotateManualVoltageRampingConstant);
        motor.config_kF(Constants.ArmConstants.rotatePID_id, Constants.ArmConstants.rotateKF);
        motor.config_kP(Constants.ArmConstants.rotatePID_id, Constants.ArmConstants.rotateKP);
        motor.config_kI(Constants.ArmConstants.rotatePID_id, Constants.ArmConstants.rotateKI);
        motor.config_kD(Constants.ArmConstants.rotatePID_id, Constants.ArmConstants.rotateKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
    }

    public double getAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.countsPerArmDegree;
        return result;
    }
    
    public void setAngle(double desiredAngle, double velocity, double accelerationTime) {
        rotateMotorLeader.configMotionCruiseVelocity(velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec);
        rotateMotorLeader.configMotionAcceleration(velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec / accelerationTime); 

        double setAngle = desiredAngle * ArmConstants.countsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public boolean isAtMaxAngle() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1 * Constants.ArmConstants.degreesPerSecToCountsPer100MSec;
    }

    public boolean isAtMinAngle() {
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1 * Constants.ArmConstants.degreesPerSecToCountsPer100MSec;
    }

    public void startRotatingAtVelocty(double velocity) { // Velocity in degrees per second
        rotateMotorLeader.set(TalonFXControlMode.Velocity, velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec);
    }

    public void setLength(double desiredLength, double velocity, double accelerationTime) {
        // inches
        // 0.0in is when arm is fully retracted

        // if (Math.abs(getArmLength() - desiredLength) >
        // Constants.ArmConstants.armLengthTolerance) {
        // extendMotor.set(Constants.ArmConstants.armExtendSpeed);
        // }

        extendMotor.configMotionCruiseVelocity(velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec / accelerationTime); 

        double setLength = desiredLength * ArmConstants.countsPerArmInch;
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength);
    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.countsPerArmInch;
        return result;
    }

    public boolean isAtMaxExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1;
    }

    public void startExtendingAtVelocty(double velocity) {
        extendMotor.set(TalonFXControlMode.Velocity, velocity * Constants.ArmConstants.degreesPerSecToCountsPer100MSec);
    }

    public void stopMoving() { // Make neutral mode for extaend and rotate motors to brake
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public String state() {
        return "returnValue";
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