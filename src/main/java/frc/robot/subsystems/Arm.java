// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase implements Sendable {
    /** Creates a new Arm. */

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings
    WPI_TalonFX extendMotor;
    WPI_TalonFX rotateMotorLeader; // 40:1 gearbox
    WPI_TalonFX rotateMotorFollower;

    double x = Constants.ArmConstants.RotateGearRatio;

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
        
        this // error is a reminder to check if clockwise inversion is correct before running the code and breaking the robot
        rotateMotorFollower.setInverted(TalonFXInvertType.Clockwise); // <-- THIS IS THE THING THAT MAY NEED TO CHANGE

        rotateMotorLeader.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);
        rotateMotorFollower.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);

        rotateMotorLeader.setSelectedSensorPosition(0);
        rotateMotorFollower.setSelectedSensorPosition(0);

        extendMotor.setSelectedSensorPosition(0);

        extendMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extendMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extendMotor.configClearPositionOnLimitR(true, 0);

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
        // NetworkTableInstance.getDefault().getEntry("arm/state").setString(this.getState().toString());
        // NetworkTableInstance.getDefault().getEntry("arm/isAtExtendLimit").setBoolean(this.isAtMaxLength());
        // NetworkTableInstance.getDefault().getEntry("arm/isAtRetractLimit").setBoolean(this.isAtMinLength());
        // NetworkTableInstance.getDefault().getEntry("arm/isAtMinRotationLimit").setBoolean(this.isAtMinAngle());
        // NetworkTableInstance.getDefault().getEntry("arm/angle").setDouble(this.getAngle());
        // NetworkTableInstance.getDefault().getEntry("arm/length").setDouble(this.getLength());
        // NetworkTableInstance.getDefault().getEntry("arm/leftMotorStatorCurrent")
        // .setDouble(rotateMotorLeader.getStatorCurrent());
        // NetworkTableInstance.getDefault().getEntry("arm/leftMotorSupplyCurrent")
        // .setDouble(rotateMotorLeader.getSupplyCurrent());
        // NetworkTableInstance.getDefault().getEntry("arm/rightMotorStatorCurrent")
        // .setDouble(rotateMotorFollower.getStatorCurrent());
        // NetworkTableInstance.getDefault().getEntry("arm/rightMotorSupplyCurrent")
        // .setDouble(rotateMotorFollower.getSupplyCurrent());
        // // System.out.println() ;
    }

    private double rotationFeedForward() {
        double ff = Constants.ArmConstants.ffRetracted + ((getLength() / Constants.ArmConstants.ArmMaxLength)
                * (Constants.ArmConstants.ffExtracted - Constants.ArmConstants.ffRetracted));
        return ff;
    }

    /* Rotate Motor */
    public void setAngle(double desiredAngle, double velocity, double accelerationTime) {
        rotateMotorLeader.configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorFollower
                .configMotionCruiseVelocity(velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec);
        rotateMotorLeader.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);
        rotateMotorFollower.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double currentAngle = getAngle();
        double ff = Math.sin(Math.toRadians(currentAngle)) * rotationFeedForward();

        if (desiredAngle > ArmConstants.ArmMaxAngle) {
            desiredAngle = ArmConstants.ArmMaxAngle;
        } else if (desiredAngle < ArmConstants.ArmMinAngle) {
            desiredAngle = ArmConstants.ArmMinAngle;
        } else if (desiredAngle < 40) { // 40 degrees is the angle between the arm support (prependicular to ground) and
                                        // the line from arm motor and the edge of the chasis
            setLength(0, Constants.ArmConstants.armRotateSpeed, Constants.ArmConstants.RotateAccel);
        }

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
        rotateMotorFollower.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
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
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, -0.07);
        rotateMotorFollower.set(TalonFXControlMode.PercentOutput, -0.07);
    }

    public void setLength(double desiredLength, double velocity, double accelerationTime) { // 0.0in is when arm is //
                                                                                            // fully retracted
        extendMotor.configMotionCruiseVelocity(
                velocity * Constants.ArmConstants.InchesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.InchesPerSecToCountsPer100MSec / accelerationTime);

        if (desiredLength > ArmConstants.ArmMaxLength) {
            desiredLength = ArmConstants.ArmMaxLength;
        } else if (desiredLength < ArmConstants.ArmMinLength) {
            desiredLength = ArmConstants.ArmMinLength;
        }

        extendMotor.configMotionCruiseVelocity(
                velocity * Constants.ArmConstants.InchesPerSecToCountsPer100MSec);
        extendMotor.configMotionAcceleration(
                velocity * Constants.ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        double ff = Constants.ArmConstants.extendFF * Math.cos(Math.toRadians(this.getAngle()));
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength, DemandType.ArbitraryFeedForward, ff);
    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean isAtMaxLength() {
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinLength() {
        return extendMotor.isRevLimitSwitchClosed() == 1;
    }

    public void startExtending(double velocity) {
        // extendMotor.set(TalonFXControlMode.Velocity, velocity *
        // Constants.ArmConstants.InchesPerSecToCountsPer100MSec);
        extendMotor.set(TalonFXControlMode.PercentOutput, -0.1); // wpk fix magic number
    }

    /* Extend and Rotate */
    public void stopMoving() {
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        rotateMotorFollower.set(TalonFXControlMode.PercentOutput, 0.0);
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    /* States */
    public void setState(Position returnValue) {
        armState = returnValue;
    }

    public Position getState() {
        return armState;
    }

    public String getStateName() {
        return armState.toString();
    }

    public double lengthLim() {
        double lengthLim; // Inches
        if (getAngle() > 29) {
            lengthLim = 0;
        } else {
            lengthLim = 37;
        } // Rough estimate, need to change
        return lengthLim;
    }

    private double getLeftStatorCurrent() {
        return rotateMotorLeader.getStatorCurrent();
    }

    private double getLeftSupplyCurrent() {
        return rotateMotorLeader.getSupplyCurrent();
    }

    private double getRightStatorCurrent() {
        return rotateMotorFollower.getStatorCurrent();
    }

    private double getRightSupplyCurrent() {
        return rotateMotorFollower.getSupplyCurrent();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Subsystem");

        builder.addStringProperty("State", this::getStateName, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Length", this::getLength, null);
        builder.addBooleanProperty("isAtRetractLimit", this::isAtMinLength, null);
        builder.addBooleanProperty("isAtExtendLimit", this::isAtMaxLength, null);
        builder.addBooleanProperty("isAtMinAngle", this::isAtMinAngle, null);

        builder.addDoubleProperty("Left Stator Current", this::getLeftStatorCurrent, null);
        builder.addDoubleProperty("Left Supply Current", this::getLeftSupplyCurrent, null);
        builder.addDoubleProperty("Right Stator Current", this::getRightStatorCurrent, null);
        builder.addDoubleProperty("Right Supply Current", this::getRightSupplyCurrent, null);
    }

    // Simulation Support

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(extendMotor, 0.1, 21777);
        PhysicsSim.getInstance().addTalonFX(rotateMotorLeader, 0.1, 21777);
        PhysicsSim.getInstance().addTalonFX(rotateMotorFollower, 0.1, 21777);
    }
}