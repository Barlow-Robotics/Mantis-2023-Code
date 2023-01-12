// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class Claw extends SubsystemBase {
    /** Creates a new Claw. */
    WPI_TalonFX clawMotor;

    public Claw() {
      clawMotor = new WPI_TalonFX(Constants.ClawConstants.clawMotorID);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setClawAngle(double angle) {
      clawMotor.set(TalonFXControlMode.MotionMagic, angle);
    }

    public double getClawAngle() {
        return 0.0;
    }

    public boolean openClaw() {
      return true;
    }
}
