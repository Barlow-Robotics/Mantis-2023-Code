// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

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

        // wpk need to set up config for motors (e.g., kF, kP, kI, kD, neutral mode, etc.)

        rotateMotorFollower.follow(rotateMotorLeader);

        rotateMotorLeader.configMotionSCurveStrength(Constants.ArmConstants.AccelerationSmoothing) ;

        extendMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen) ;
        extendMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen) ;
        // wpk, should we also use the TalonFX built in "soft" limit switches?

        // wpk, we can automatically reset the encoder when the retracted (reverse) limit is reached. Do we want to do this?
        // If so, it would look like this:
        extendMotor.configClearPositionOnLimitR(true, 0) ;

        // wpk - the same configuration needs to be set up for the arm angle limits.
        // Please fill this in.


    }

    @Override
    public void periodic() {

        // /* if button is up, just drive the motor in PercentOutput */
        // if (robot.currentProfileButton == false) {
        //     state = 0;
        // }

        // switch (state) {
        //     /* drive extendMotor talon normally */
        //     case 0:
        //         extendMotor.set(TalonFXControlMode.Velocity, Constants.ArmConstants.armRotateSpeed);
        //         if (robot.currentProfileButton == true) {
        //             /* go to MP logic */
        //             state = 1;
        //         }
        //         break;

        //     /* fire the MP, and stop calling set() since that will cancel the MP */
        //     case 1:
        //         /* wait for 10 points to buffer in firmware, then transition to MP */
        //         extendMotor.startMotionProfile(bufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        //         state = 2;
        //         // Instrum.printLine("MP started");
        //         break;

        //     /* wait for MP to finish */
        //     case 2:
        //         if (extendMotor.isMotionProfileFinished()) {
        //             // Instrum.printLine("MP finished");
        //             state = 3;
        //         }
        //         break;

        //     /* MP is finished, nothing to do */
        //     case 3:
        //         break;
        // }

        /* print MP values */
        // Instrum.loop(bPrintValues, _master);
    }

    public double getArmAngle() {
        double result = rotateMotorLeader.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmDegree;
        return result;
    }


    // wpk Can probably remove "Arm" from all/most of the methods since we know this is an arm and having "Arm" again
    // is redundant and makes the names longer.

    public void setArmAngle(double desiredAngle, double cruiseVelocity, double acceleration) {
        // degrees

        // if (Math.abs(getArmAngle() - desiredAngle) > Constants.ArmConstants.armAngleTolerance) {
        //     rotateMotorLeader.set(Constants.ArmConstants.armRotateSpeed);
        // }

        rotateMotorLeader.configMotionCruiseVelocity(cruiseVelocity) ;
        rotateMotorLeader.configMotionAcceleration(acceleration) ;

        double setAngle = desiredAngle * ArmConstants.CountsPerArmDegree;
        rotateMotorLeader.set(TalonFXControlMode.MotionMagic, setAngle);

        // *** need to make it so the robot can't extend if the claw is facing down (or if the angle is between a certain range)
    }

    public boolean IsAtMaxAngle() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1 ;
    }

    public boolean IsAtMinAngle() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1 ;
    }


    public void StartRotatingAtVelocty(double velocity) {

        // wpk - need to fill this in. This is to support calibrating the arm angle by "slowly" lowing it until the limit switch is
        // reached. This will be used in conjuction with "IsAtMinAngle" by a command to calibrate the arm position.

        // Please fill in this code...
    }





    public void setArmLength(double desiredLength, double velocity, double acceleration) {
        // inches
        // 0.0in is when arm is fully retracted

        // if (Math.abs(getArmLength() - desiredLength) > Constants.ArmConstants.armLengthTolerance) {
        //     extendMotor.set(Constants.ArmConstants.armExtendSpeed);
        // }

        // wpk need to set velocity and acceleration. See setArmAngle for example.

        double setLength = desiredLength * ArmConstants.CountsPerArmInch;
        extendMotor.set(TalonFXControlMode.MotionMagic, setLength);
    }

    public double getArmLength() {
        double result = extendMotor.getSelectedSensorPosition() / Constants.ArmConstants.CountsPerArmInch;
        return result;
    }

    public boolean IsAtMaxExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isFwdLimitSwitchClosed() == 1 ;
    }

    public boolean IsAtMinExtension() {
        // wpk need to read and return the value of the limit switch
        return rotateMotorLeader.isRevLimitSwitchClosed() == 1 ;
    }


    public void StartExtendingAtVelocty(double velocity) {

        // wpk - need to fill this in. This is to support calibrating the arm extension by "slowly" retracting it until the limit switch is
        // reached. This will be used in conjuction with "IsAtMinExtension" by a command to calibrate the arm extension.

        // Please fill in this code...
    }
 


    public void StopMoving() {
        rotateMotorLeader.set(TalonFXControlMode.PercentOutput, 0.0) ;
        extendMotor.set(TalonFXControlMode.PercentOutput, 0.0) ;
    }


    public void startProfiles() {
        rotateMotorLeader.startMotionProfile(null, 0, ControlMode.MotionProfile); // Rotate profile
        extendMotor.startMotionProfile(null, 0, ControlMode.MotionProfile); // Extend profile
    }

    public boolean isProfileComplete() {
        return extendMotor.isMotionProfileFinished();
    }
}