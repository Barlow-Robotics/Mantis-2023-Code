// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    /** Creates a new Claw. */

    WPI_TalonFX clawMotor; // For adjusting angle
    Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.ExtendSolenoidID);
    Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.RetractSolenoidID);
    TimeOfFlight distanceSensor = new TimeOfFlight(Constants.ClawConstants.DistanceSensorID);

    // Add distance sensor from playing with fusion

    Arm armSub;
    // private final Timer timer = new Timer();
    
    boolean autoCloseEnabled = true;

    public Claw(Arm a) { // add arm to constructors
        armSub = a;
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.ClawMotorID); // needs config

        setClawMotorConfig(clawMotor);
    }

    private void setClawMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configClosedloopRamp(Constants.ClawConstants.ClawClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ClawConstants.ClawManualVoltageRampingConstant);
        motor.config_kF(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKF);
        motor.config_kP(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKP);
        motor.config_kI(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKI);
        motor.config_kD(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // 0.0 is perpendicular to arm bar
        setAngle(90 - armSub.getAngle()); // Probably doesnt work (?)
        
        if (isOpen() && autoCloseEnabled && distanceSensor.getRange() <= (ClawConstants.InchesForAutoClosing) * Constants.InchesToMillimeters) {
            close();
            disableAutoClose();
        }
        else if (isOpen() && distanceSensor.getRange() >= (ClawConstants.ClawLengthInches) * Constants.InchesToMillimeters) {
            enableAutoClose();
        }
    }

    public double getAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.CountsPerClawDegree;
        return result;
    }

    public void setAngle(double desiredAngle) {
        double setAngle = desiredAngle * ClawConstants.CountsPerClawDegree; 
        clawMotor.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public void open() {
        extendSolenoid.set(false);
        retractSolenoid.set(true);
    }

    public void close() {
        retractSolenoid.set(false);
        extendSolenoid.set(true);
    }

    public boolean isOpen() {
        return retractSolenoid.get() && !extendSolenoid.get();
    }

    public void enableAutoClose() {
        autoCloseEnabled = true;
    }

    public void disableAutoClose() {
        autoCloseEnabled = false;
    }
}