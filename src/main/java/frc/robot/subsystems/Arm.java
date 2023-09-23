// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.sim.PhysicsSim;

import org.littletonrobotics.junction.Logger;


public class Arm extends SubsystemBase implements Sendable {
    /** Creates a new Arm. */

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings
    WPI_TalonFX extendMotor;
    WPI_TalonFX leftRotateMotor;
    WPI_TalonFX rightRotateMotor;
    
    WPI_CANCoder rotationEncoderL;

    double x = Constants.ArmConstants.RotateGearRatio;

    public enum Position {
        Home, Bottom, Middle, Top, Floor, PlayerStation, Transition
    };

    public Position armState = Position.Home;

    // BufferedTrajectoryPointStream bufferedStream = new
    // BufferedTrajectoryPointStream();

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.ArmExtendMotorID);
        leftRotateMotor = new WPI_TalonFX(Constants.ArmConstants.LeftArmMotorID);
        rightRotateMotor = new WPI_TalonFX(Constants.ArmConstants.RightArmMotorID);
        rotationEncoderL = new WPI_CANCoder(Constants.ArmConstants.CANCoderID) ;

        setExtendMotorConfig(extendMotor);

        setRotateMotorConfig(leftRotateMotor);
        setRotateMotorConfig(rightRotateMotor);

        rightRotateMotor.setInverted(TalonFXInvertType.Clockwise); // maybe change

        leftRotateMotor.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);
        rightRotateMotor.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing);

        leftRotateMotor.setSelectedSensorPosition(0);
        // rotationEncoderL.setPosition(0);
        rightRotateMotor.setSelectedSensorPosition(0);

        extendMotor.setSelectedSensorPosition(0);

        extendMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extendMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        extendMotor.configClearPositionOnLimitR(true, 0);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180 ;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition ;
        // canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero ;
        canCoderConfiguration.sensorDirection = true ;
        canCoderConfiguration.magnetOffsetDegrees = -174.1 ;
        rotationEncoderL.configAllSettings(canCoderConfiguration);

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
        leftRotateMotor.configMotionCruiseVelocity(velocity * ArmConstants.DegreesPerSecToCountsPer100MSec);
        rightRotateMotor.configMotionCruiseVelocity(velocity * ArmConstants.DegreesPerSecToCountsPer100MSec);
        leftRotateMotor.configMotionAcceleration( velocity * ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);
        rightRotateMotor.configMotionAcceleration( velocity * ArmConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        double currentAngle = getAngle();
        double ff = Math.sin(Math.toRadians(currentAngle)) * rotationFeedForward();

        if (desiredAngle > ArmConstants.ArmMaxAngle) {
            desiredAngle = ArmConstants.ArmMaxAngle;
        } else if (desiredAngle < ArmConstants.ArmMinAngle) {
            desiredAngle = ArmConstants.ArmMinAngle;
        } else if (desiredAngle < 40) { // 40 degrees is the angle between the arm support (prependicular to ground) and
                                        // the line from arm motor and the edge of the chasis
            setLength(0, ArmConstants.ArmExtendSpeed, ArmConstants.ArmExtendAccelerationTime);
        }

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        NetworkTableInstance.getDefault().getEntry("arm/setAngle").setDouble(setAngle);
        NetworkTableInstance.getDefault().getEntry("arm/desired_angle").setDouble(desiredAngle);

        //  System.out.println("Setting arm angle to " + desiredAngle + "( " +  setAngle + " ) with feed forward "+ ff ) ;
        leftRotateMotor.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
        rightRotateMotor.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
    }

    public double getAngle() {
        double result = leftRotateMotor.getSelectedSensorPosition() / ArmConstants.CountsPerArmDegree ;
        // double result = rotationEncoderL.getAbsolutePosition() / ArmConstants.CountsPerArmDegree;
        return result;
    }

    public boolean isAtMaxAngle() {
        return leftRotateMotor.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinAngle() {
        return leftRotateMotor.isRevLimitSwitchClosed() == 1;
    }

    public void startRotating(double velocity) { // Velocity in degrees per second
        leftRotateMotor.set(TalonFXControlMode.PercentOutput, -0.07);
        rightRotateMotor.set(TalonFXControlMode.PercentOutput, -0.07);
    }

    public void setLength(double desiredLength, double velocity, double accelerationTime) { // 0.0in is when arm is //
                                                                                            // fully retracted
        extendMotor.configMotionCruiseVelocity(velocity * ArmConstants.InchesPerSecToCountsPer100MSec);
       extendMotor.configMotionAcceleration(velocity * ArmConstants.InchesPerSecToCountsPer100MSec / accelerationTime);

        if (desiredLength > ArmConstants.ArmMaxLength) {
            desiredLength = ArmConstants.ArmMaxLength;
        } else if (desiredLength < ArmConstants.ArmMinLength) {
            desiredLength = ArmConstants.ArmMinLength;
        }

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        double ff = ArmConstants.ExtendFF * Math.cos(Math.toRadians(this.getAngle()));
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength, DemandType.ArbitraryFeedForward, ff);
    
        NetworkTableInstance.getDefault().getEntry("arm/setLength").setDouble(setLength);
        NetworkTableInstance.getDefault().getEntry("arm/desiredLength").setDouble(desiredLength);

    }

    public double getLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean isAtMaxLength() {
        return leftRotateMotor.isFwdLimitSwitchClosed() == 1;
    }

    public boolean isAtMinLength() {
        return extendMotor.isRevLimitSwitchClosed() == 1;
    }

    public void startExtending(double velocity) {
        // extendMotor.set(TalonFXControlMode.Velocity, velocity *
        // Constants.ArmConstants.InchesPerSecToCountsPer100MSec);
        extendMotor.set(TalonFXControlMode.PercentOutput, -0.2); // wpk fix magic number
    }

    /* Extend and Rotate */
    public void stopMoving() {
        leftRotateMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rightRotateMotor.set(TalonFXControlMode.PercentOutput, 0.0);
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
        return leftRotateMotor.getStatorCurrent();
    }

    private double getLeftSupplyCurrent() {
        return leftRotateMotor.getSupplyCurrent();
    }

    private double getRightStatorCurrent() {
        return rightRotateMotor.getStatorCurrent();
    }

    private double getRightSupplyCurrent() {
        return rightRotateMotor.getSupplyCurrent();
    }

    private double getLeftError() {
        return this.leftRotateMotor.getClosedLoopError();
        // return this.rotationEncoderL.geterror;  //??? -Ang
    }

    private double getRightError() {
        return this.rightRotateMotor.getClosedLoopError() ;
    }

    private double getExtendError() {
        return this.extendMotor.getClosedLoopError() ;
    }

    private double getLeftVelocity() {
        return this.leftRotateMotor.getSelectedSensorVelocity() ;
    }

    private double getExtendVelocity() {
        return this.extendMotor.getSelectedSensorVelocity() ;
    }

    // public void setMotorDefaults() {
    //     rotationEncoderL.restoreFactoryDefaults();
    //     rotationEncoderL.setIdleMode(IdleMode.kBrake);
    // }

    private double getAbsoluteEncoderAngle() {
        return rotationEncoderL.getAbsolutePosition() ;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Subsystem");

        builder.addStringProperty("State", this::getStateName, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Absolute Encoder Angle", this::getAbsoluteEncoderAngle, null);
        builder.addDoubleProperty("Length", this::getLength, null);
        builder.addBooleanProperty("isAtRetractLimit", this::isAtMinLength, null);
        builder.addBooleanProperty("isAtExtendLimit", this::isAtMaxLength, null);
        builder.addBooleanProperty("isAtMinAngle", this::isAtMinAngle, null);

        builder.addDoubleProperty("Left Closed Loop Error", this::getLeftError, null);
        builder.addDoubleProperty("Right Closed Loop Error", this::getRightError, null);
        builder.addDoubleProperty("Extend Closed Loop Error", this::getExtendError, null);
        builder.addDoubleProperty("Left Motor Velocity", this::getLeftVelocity, null);
        builder.addDoubleProperty("Extend Motor Velocity", this::getExtendVelocity, null);
    }

    // Simulation Support

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(extendMotor, 0.1, 21777);
        PhysicsSim.getInstance().addTalonFX(leftRotateMotor, 0.1, 21777);
        PhysicsSim.getInstance().addTalonFX(rightRotateMotor, 0.1, 21777);
    }
}