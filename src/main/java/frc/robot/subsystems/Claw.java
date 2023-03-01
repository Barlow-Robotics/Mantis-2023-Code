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
            Constants.ClawConstants.extendSolenoidID);
    Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.retractSolenoidID);
    TimeOfFlight distanceSensor = new TimeOfFlight(Constants.ClawConstants.distanceSensorID);

    // Add distance sensor from playing with fusion

    Arm armSub;
    // private final Timer timer = new Timer();

    boolean autoCloseEnabled = true;

    public Claw(Arm a) { // add arm to constructors
        armSub = a;
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.clawMotorID); // needs config

        setClawMotorConfig(clawMotor);
    }

    private void setClawMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configClosedloopRamp(Constants.ClawConstants.clawClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ClawConstants.clawManualVoltageRampingConstant);
        motor.config_kF(Constants.ClawConstants.clawPID_id, Constants.ClawConstants.clawKF);
        motor.config_kP(Constants.ClawConstants.clawPID_id, Constants.ClawConstants.clawKP);
        motor.config_kI(Constants.ClawConstants.clawPID_id, Constants.ClawConstants.clawKI);
        motor.config_kD(Constants.ClawConstants.clawPID_id, Constants.ClawConstants.clawKD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // 0.0 is perpendicular to arm bar
        setClawAngle(90 - armSub.getAngle()); // Probably doesnt work (?)
        
        if (clawIsOpen() && autoCloseEnabled && distanceSensor.getRange() <= (ClawConstants.inchesForAutoClosing) * Constants.InchesToMillimeters) {
            closeClaw();
            disableAutoClose();
        }
        else if (clawIsOpen() && distanceSensor.getRange() >= (ClawConstants.clawLengthInches) * Constants.InchesToMillimeters) {
            enableAutoClose();
        }

    }

    public double getClawAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.countsPerClawDegree;
        return result;
    }

    public void setClawAngle(double desiredAngle) {
        // if (Math.abs(getClawAngle() - desiredAngle) >
        // ClawConstants.ClawAngleTolerance) {
        // clawMotor.set(Constants.ClawConstants.clawSpeed);
        // }
        double setAngle = desiredAngle * ClawConstants.countsPerClawDegree; 
        clawMotor.set(TalonFXControlMode.MotionMagic, setAngle);
    }

    public void openClaw() {
        extendSolenoid.set(false);
        retractSolenoid.set(true);
    }

    public void closeClaw() {
        retractSolenoid.set(false);
        extendSolenoid.set(true);
    }

    public boolean clawIsOpen() {
        return retractSolenoid.get() && !extendSolenoid.get();
    }

    public void enableAutoClose() {
        autoCloseEnabled = true;
    }

    public void disableAutoClose() {
        autoCloseEnabled = false;
    }

    // public void toggleAutoClose() {
    //     autoCloseEnabled = !autoCloseEnabled;
    // }
}
