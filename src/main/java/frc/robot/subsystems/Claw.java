// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Arm.Position;

public class Claw extends SubsystemBase {

    WPI_TalonFX clawMotor; // For adjusting angle

    // Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    Solenoid closeSolenoid;
    Solenoid openSolenoid;

    TimeOfFlight distanceSensor = new TimeOfFlight(Constants.ClawConstants.DistanceSensorID);

    // Add distance sensor from playing with fusion

    Arm armSub;

    boolean autoCloseEnabled = true;
    boolean open = false;

    double targetAngle = 0.0 ;

    public Claw(Arm a) { // add arm to constructors
        closeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.CloseSolenoidID);
        openSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.OpenSolenoidID);


        armSub = a;
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.ClawMotorID); // needs config

        setClawMotorConfig(clawMotor);
        clawMotor.setSelectedSensorPosition(0);

        distanceSensor.setRangingMode(RangingMode.Short, 24);

        this.close();
    }

    private void setClawMotorConfig(WPI_TalonFX motor) {
        motor.configClosedloopRamp(Constants.ClawConstants.ClawClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ClawConstants.ClawManualVoltageRampingConstant);
        motor.config_kF(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKF);
        motor.config_kP(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKP);
        motor.config_kI(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKI);
        motor.config_kD(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKD);

        motor.setInverted(TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Brake);

        // motor.configMotionCruiseVelocity(
        //         Constants.ClawConstants.ClawSpeed * Constants.ClawConstants.DegreesPerSecToCountsPer100MSec);
        // motor.configMotionAcceleration(Constants.ClawConstants.DegreesPerSecToCountsPer100MSec / 0.1); // wpk add a
        //                                                                                                // constant for
        //                                                                                                // this one

        motor.configMotionSCurveStrength(Constants.ClawConstants.AccelerationSmoothing);

        // wpk add something for soft limits.

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
        // 0.0 is perpendicular to arm bar

        final double maxDelta = 20.0 ;
        double delta = 0.0 ;
        double armAngle = armSub.getAngle() ;

        
        if (armSub.getState() == Position.PlayerStation) {
            delta = -1.5 ;
        } else if ( armAngle > (ArmConstants.MiddleArmAngle - 5)) {    
            // if the arm is up high, gradually raise claw angle
            double percentOfDelta = ( armAngle - (ArmConstants.MiddleArmAngle - 5) ) / ( ArmConstants.TopArmAngle - (ArmConstants.MiddleArmAngle - 5) ) ;
            delta = percentOfDelta * maxDelta ;
        // } else if (armAngle < ArmConstants.FloorArmAngle) {
        } else if (armAngle < 3 || armSub.getState() == Position.Floor) {
            // if arm is nearing home position, lower claw slightly so motor doesn't stall against claw stops
            delta = -1.0 ;
        } else {
            // otherwise, keep claw level with the floor
            delta = 2.0 ;
        }
        SmartDashboard.putNumber("Claw Delta", delta);
        setAngle((-armAngle + delta), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        

        // if ( armSub.getAngle() > (ArmConstants.TopArmAngle - 5)) {
        //     setAngle((-armSub.getAngle() + 20.0), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        // } else if (armSub.getAngle() > 5) {
        //     // setAngle((-armSub.getAngle() + 2.0), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        //     setAngle((-armSub.getAngle() + 0.0), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        // } else {
        //     // setAngle((-armSub.getAngle() + 2.0), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        //     setAngle((-armSub.getAngle() + 0.0), Constants.ArmConstants.RotateVel, Constants.ArmConstants.RotateAccel);
        // }

        if (isOpen() && autoCloseEnabled
                && distanceSensor.isRangeValid()
                && distanceSensor.getRange() <= (ClawConstants.InchesForAutoClosing) * Constants.InchesToMillimeters) {
            close();
            disableAutoClose();
        } else if (distanceSensor.isRangeValid() && distanceSensor.getRange() >= 15 * Constants.InchesToMillimeters)
                     {
            enableAutoClose();
        } 
    }

    public double getAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.CountsPerClawDegree;
        return result;
    }

    public void setAngle(double desiredAngle, double velocity, double accelerationTime) {
        clawMotor.configMotionCruiseVelocity( velocity * Constants.ClawConstants.DegreesPerSecToCountsPer100MSec);
        // wpk add constant for acceleration 
        clawMotor.configMotionAcceleration(Constants.ClawConstants.DegreesPerSecToCountsPer100MSec / 0.1); 

        targetAngle = desiredAngle ;
        double setAngle = desiredAngle * ClawConstants.CountsPerClawDegree;
        // clawMotor.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, Constants.ClawConstants.ff );
        clawMotor.set(TalonFXControlMode.Position, setAngle, DemandType.ArbitraryFeedForward, 0.1);
    }

    public void startMoving() {
        clawMotor.set(TalonFXControlMode.PercentOutput, 0.1);
    }

    public void stopMoving() {
        clawMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public double getOutput() {
       return clawMotor.getMotorOutputPercent();
    }

    public void close() {
        openSolenoid.set(true);
        closeSolenoid.set(false);
        open = false;
    }

    public void open() {
        openSolenoid.set(false);
        closeSolenoid.set(true);
        open = true;
    }

    public boolean isOpen() {
        return open;
    }

    public boolean getAutoCloseEnable() {
        return this.autoCloseEnabled ;
    }

    public void setAutoCloseEnabled( boolean value ) {
        this.autoCloseEnabled = value ;
    }

    public void enableAutoClose() {
        autoCloseEnabled = true;
    }

    public void disableAutoClose() {
        autoCloseEnabled = false;
    }

    public double getDistanceInches() {
        return (distanceSensor.getRange() / Constants.InchesToMillimeters);
    }

    public TimeOfFlight getRangeSensor() {
        return this.distanceSensor;
    }

    public boolean getRangeIsValid() {
        return this.distanceSensor.isRangeValid() ;
    }

    private double getClosedLoopError() {
        return this.clawMotor.getClosedLoopError() / Constants.ClawConstants.CountsPerClawDegree ;
    }

    public double getSupplyCurrent() {
        return this.clawMotor.getSupplyCurrent();
    }

    public double getStatorCurrent() {
        return this.clawMotor.getStatorCurrent();
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public void resetEncoders() {
        this.clawMotor.setSelectedSensorPosition(0);
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Claw Subsystem");
    
        builder.addBooleanProperty("Open", this::isOpen, null);
        builder.addBooleanProperty("Range Valid", this::getRangeIsValid, null);
        builder.addDoubleProperty("Range", this::getDistanceInches, null);
        builder.addBooleanProperty("Auto Close Enable", this::getAutoCloseEnable, this::setAutoCloseEnabled);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Target Angle", this::getTargetAngle, null);
        builder.addDoubleProperty("Error", this::getClosedLoopError, null);
        builder.addDoubleProperty("Supply Current", this::getSupplyCurrent, null);
        builder.addDoubleProperty("Percent Output", this::getOutput, null);
    }


    // Simulation Support

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(clawMotor, 0.1, 21777);
    }



}