// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase { // Extend, move to a certain place,
    /** Creates a new Arm. */

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings
    WPI_TalonFX extendMotor;
    WPI_TalonFX rotateMotorLeader; // 40:1 gearbox
    WPI_TalonFX rotateMotorFollower;

    double x = Constants.ArmConstants.rotateGearRatio;

    private RobotContainer robotContainer;
    private Robot robot;

    int state = 0;

    BufferedTrajectoryPointStream bufferedStream = new BufferedTrajectoryPointStream();

    public Arm() {
        extendMotor = new WPI_TalonFX(Constants.ArmConstants.armExtendMotorID);
        rotateMotorLeader = new WPI_TalonFX(Constants.ArmConstants.armLeaderMotorID);
        rotateMotorFollower = new WPI_TalonFX(Constants.ArmConstants.armFollowMotorID);

        rotateMotorFollower.follow(rotateMotorLeader);
    }

    @Override
    public void periodic() {

        /* if button is up, just drive the motor in PercentOutput */
        if (robot.currentProfileButton == false) {
            state = 0;
        }

        switch (state) {
            /* drive extendMotor talon normally */
            case 0:
                extendMotor.set(TalonFXControlMode.Velocity, Constants.ArmConstants.armRotateSpeed);
                if (robot.currentProfileButton == true) {
                    /* go to MP logic */
                    state = 1;
                }
                break;

            /* fire the MP, and stop calling set() since that will cancel the MP */
            case 1:
                /* wait for 10 points to buffer in firmware, then transition to MP */
                extendMotor.startMotionProfile(bufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
                state = 2;
                // Instrum.printLine("MP started");
                break;

            /* wait for MP to finish */
            case 2:
                if (extendMotor.isMotionProfileFinished()) {
                    // Instrum.printLine("MP finished");
                    state = 3;
                }
                break;

            /* MP is finished, nothing to do */
            case 3:
                break;
        }

        /* print MP values */
        // Instrum.loop(bPrintValues, _master);
    }

    public double getArmAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.UnitsPerArmDegree;
        return result;
    }

    public void setArmAngle(double desiredAngle) {
        // degrees

        if (Math.abs(getArmAngle() - desiredAngle) > Constants.ArmConstants.armAngleTolerance) {
            rotateMotorLeader.set(Constants.ArmConstants.armRotateSpeed);
        }
    }

    public void setArmLength(double desiredLength) {
        // inches
        // 0.0in is when arm is fully retracted

        if (Math.abs(getArmLength() - desiredLength) > Constants.ArmConstants.armLengthTolerance) {
            extendMotor.set(Constants.ArmConstants.armExtendSpeed);
        }
    }

    public double getArmLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.UnitsPerArmInch;
        return result;
    }

    public void startProfiles() {
        rotateMotorLeader.startMotionProfile(null, 0, ControlMode.MotionProfile); // Rotate profile
        extendMotor.startMotionProfile(null, 0, ControlMode.MotionProfile); // Extend profile
    }

    public boolean isProfileComplete() {
        return extendMotor.isMotionProfileFinished();
    }
}