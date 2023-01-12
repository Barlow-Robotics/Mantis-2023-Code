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


public class Arm extends SubsystemBase {      // Extend, move to a certain place, 
  /** Creates a new Arm. */

  WPI_TalonFX armTelescopeMotor;
  WPI_TalonFX armRotateMotor;

  public Arm() {
    armTelescopeMotor = new WPI_TalonFX(Constants.ArmConstants.armTelescopeMotorID);
    armRotateMotor = new WPI_TalonFX(Constants.ArmConstants.armRotateMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  public void setArmAngle(double desiredAngle) {
    armTelescopeMotor.set(TalonFXControlMode.MotionMagic, desiredAngle);
  }

  public double getArmAngle(){
    return 0.0;
  }

  public void setArmInches(double desiredInches){
  }

  public double getArmInches(){
    return 0.0;
  } 
} 
