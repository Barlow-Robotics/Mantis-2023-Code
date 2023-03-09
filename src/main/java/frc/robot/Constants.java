// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double InchesToMeters = 0.0254;
    public static final double SecondsTo100MSec = 0.1;
    public static final double InchesToMillimeters = InchesToMeters * 1000;

    public static final double TalonFXEncoderResolution = 2048;
    public static final double TalonSRXEncoderResolution = 4096;

    public final static double kNeutralDeadband = 0.001;

    public static final double maxGravityFF = 0.07;

    // PID Gains may have to be adjusted based on the responsiveness of control loop
    // kP kI kD kF Iz PeakOut
    public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);
    // measured 6800 velocity units at full motor output

    public final static int kPrimaryPIDSlot = 0; // any slot [0,3]

    public static final class DriveConstants {
        public static final int DriveMotorLeftLeaderID = 4;
        public static final int DriveMotorLeftFollowerID = 5;
        public static final int DriveMotorRightLeaderID = 6;
        public static final int DriveMotorRightFollowerID = 7;

        public static final double MaxSpeed = 4.0; // meters per second
        // public static final double maxAngularSpeed = 2 * Math.PI; // one rotation per
        // second

        public static final double TrackWidth = 26.5 * InchesToMeters; // meters

        public static final double DriveGearRatio = 9.8;
        public static final double CountsPerWheelRevolution = TalonFXEncoderResolution * DriveGearRatio;

        public static final double WheelDiameter = 6.0 * InchesToMeters;
        public static final double MetersPerRevolution = WheelDiameter * Math.PI;
        public static final double RevolutionsPerMeter = 1.0 / MetersPerRevolution;

        public static final double MetersPerSecondToCountsPerSecond = RevolutionsPerMeter * CountsPerWheelRevolution;
        public static final double MaxSpeedCountsPer100MSec = MaxSpeed * MetersPerSecondToCountsPerSecond / 10.0;

        // public static final double metersPerCount = metersPerRevolution /
        // countsPerWheelRevolution;
        // public static final double CountsPerMeterPerSecond = 1 / metersPerCount;
        public static final double CorrectionRotationSpeed = 2.0; // Arbitrarily assigned, need to change w/ testing

        public static final double ClosedVoltageRampingConstant = 0.0;
        public static final double ManualVoltageRampingConstant = 0.0;
        public static final double kF = 0.048;
        public static final double kP = 0.005;
        public static final double kI = 0.0001;
        public static final double kD = 0.0;
        public static final int PID_id = 0;

        public static final double BalanceTolerance = 2.5; 
    }

    public static final class ArmConstants {
        public static final double RotateGearRatio = 40;
        public static final double ExtendGearRatio = 16;

        public static final double ExtendSprocketDiameter = 1.751;
        public static final double ExtendSprocketCircumference = ExtendSprocketDiameter * Math.PI;

        public static final double InchesPerRevolution = ExtendSprocketCircumference / ExtendGearRatio;
        public static final double RevolutionsPerDegree = 1.0 / 360; // Is this right?

        // add values when we figure out lengths
        public static final double CountsPerArmDegree = TalonFXEncoderResolution * RevolutionsPerDegree
                * RotateGearRatio;
        public static final double CountsPerArmInch = TalonFXEncoderResolution / InchesPerRevolution;

        public static final double InchesPerSecToCountsPer100MSec = CountsPerArmInch * SecondsTo100MSec;
        public static final double DegreesPerSecToCountsPer100MSec = CountsPerArmDegree * SecondsTo100MSec;

        public static final double armRotateSpeed = 55; // Degrees per second
        public static final double armRotateAccelerationTime = 0.25; 
        public static final double armExtendSpeed = 25; // Inches per second
        public static final double armExtendAccelerationTime = 0.25; // Need to change this (time in sceonds for total motion)

        public static final double ExtentionCalibrationVelocity = -1.0; 
        public static final double RotationCalibrationVelocity = 10; // Degrees per second

        public static final int ArmExtendMotorID = 12;
        public static final int ArmLeaderMotorID = 11;
        public static final int ArmFollowMotorID = 10;

        public static final int AccelerationSmoothing = 2;

        public static final double ArmAngleTolerance = 1; 
        public static final double ArmLengthTolerance = 0.5; 

        // wpk need to add constants for slow and fast velocities for moving the arm.

        /* Automatic Arm Positioning Constants */

        public static final double RestingArmAngle = 0.0;
        public static final double RestingArmLength = 0.0;

        public static final double RestingFromFloorArmAngle = 50.0;

        public static final double FloorArmAngle = 40.0;
        public static final double FloorArmLength = 6.0;  // inches

        public static final double PlayerStationArmAngle = 93.0;
        public static final double PlayerStationArmLength = 0.0;

        public static final double TopArmAngle = 110.0;
        public static final double TopArmLength = 0.7;

        public static final double MiddleArmAngle = 100.0;
        public static final double MiddleArmLength = 0.05;

        public static final double MiddleFromBottomArmAngle = 100.0;
        public static final double MiddleFromBottomArmLength = 0.0;

        public static final double BottomArmAngle = 30.0;
        public static final double BottomArmLength = 0.3;

        public static final double AvoidChassisArmAngle = 30.0;
        public static final double AvoidChassisArmLength = 0.0;

        public static final double ArmMinAngle = 0.0;
        public static final double ArmMaxAngle = 120.0;
        public static final double ArmMinLength = 0.0;
        public static final double ArmMaxLength = 26.0; // need to confirm (inches)

        public static final double AngleVel = 50;
        public static final double AngleAccelerationTime = 0.25;
        public static final double AngleMultiplier = 1.0;
        // public static final double AngleVel = 0.5;
        public static final double AngleAcceleration = AngleVel * 4.0;
        // public static final double AngleMultiplier = 0.5;

        public static final double LengthVel = 30.0;
        public static final double LengthAccelTime = 0.25;
        public static final double LengthMultiplier = 0.5;

        // wpk will probably need to add constants for minimum angle before retracting
        // arm. This is required to avoid crashing the claw into the chassis when going
        // to home from a position where the claw is above the chassis.

        // Not sure, but these (124-130) might need to change (right now they're just
        // taken from DriveConstants):
        public static final double RotateClosedVoltageRampingConstant = 0.0;
        public static final double RotateManualVoltageRampingConstant = 0.0;
        public static final double RotateKF = 0.048;
        // public static final double RotateKF = 0.1;
        public static final double RotateKP = 0.22;
        public static final double RotateKI = 0.0001;
        public static final double RotateKD = 1.2;
        public static final int RotatePID_id = 0;

        public static final double LengthClosedVoltageRampingConstant = 0.0;
        public static final double LengthManualVoltageRampingConstant = 0.0;
        public static final double LengthKF = 0.048;
        public static final double LengthKP = 0.1;
        public static final double LengthKI = 0.001;
        public static final double LengthKD = 0.1;
        public static final int LengthPID_id = 0;

        public static final double ffRetracted = 0.09; // Need to re-test to find this after claw is attached
        public static final double ffExtracted = 0.20; // Need to re-test to find this after claw is attached

        // public static final int kMeasuredPosHorizontal = 840; // ALH - this is
        // supposed to be position measured when arm is horizontal, need to determine
        // what that is.

    }

    public static final class ClawConstants {
        
        public static final int ClawMotorID = 15;
        public static final int ExtendSolenoidID = 4;
        public static final int RetractSolenoidID = 5;

        public static final double RotateGearRatio = 32.861; // Need to change

        public static final double RevolutionsPerDegree = 1.0 / 360.0;

        public static final double ClawClosedVoltageRampingConstant = 0.0;
        public static final double ClawManualVoltageRampingConstant = 0.0;
        public static final double ClawKF = 0.048;
        public static final double ClawKP = 0.005;
        public static final double ClawKI = 0.0000;
        public static final double ClawKD = 0.0;
        public static final int ClawPID_id = 0;
        public static final double ff = 0.0 ;

        public static final double CountsPerClawDegree = TalonFXEncoderResolution * RevolutionsPerDegree
            * RotateGearRatio; // Need to change

        public static final double ClawAngleTolerance = 1.0 ; 

        public static final double ClawSpeed = 30.0 ; // Needs tuning
        public static final int DistanceSensorID = 0; // Need to change

        public static final double InchesForAutoClosing = 3;
        public static final double ClawLengthInches = 9;
    }

    public static final class UnderGlowConstants {
        public static final SerialPort.Port Port = SerialPort.Port.kUSB1;
        public static final int BlueAliance = 1;
        public static final int RedAliance = 2;
        public static final int NeonGreen = 3;
    }

    public static final class VisionConstants {
        public static final int CameraLightID = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.25;
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond / 0.5;
    }

    public final class LogitechDualActionConstants {
        public static final int LeftJoystickX = 0;
        public static final int LeftJoystickY = 1;
        public static final int RightJoystickX = 2;
        public static final int RightJoystickY = 3;

        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int ButtonA = 2;
        public static final int ButtonB = 3;
        public static final int ButtonX = 1;
        public static final int ButtonY = 4;
        public static final int BackButton = 9;
        public static final int StartButton = 10;
        public static final int LeftJoystick = 11;
        public static final int RightJoystick = 12;

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class RadioMasterConstants {
        public static final int LeftGimbalX = 0;
        public static final int LeftGimbalY = 1;
        public static final int RightGimbalX = 3;
        public static final int RightGimbalY = 2;

        public static final int SliderF = 5;
        public static final int SliderE = 4;
        public static final int SliderC = 6;

        public static final int ButtonD = 3;
        public static final int ButtonA = 1;

        public static final double FowardAxisAttenuation = 1.0;
        public static final double LateralAxisAttenuation = 1.0;
        public static final double YawAxisAttenuation = 0.6;
    }

    public final class XboxControllerConstants {
        public static final int LeftStickX = 0;
        public static final int LeftStickY = 1;
        public static final int LeftTrigger = 2;
        public static final int RightTrigger = 4;
        public static final int RightStickX = 4;
        public static final int RightStickY = 5;

        public static final int ButtonA = 1;
        public static final int ButtonB = 2;
        public static final int ButtonX = 3;
        public static final int ButtonY = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int BackButton = 7;
        public static final int StartButton = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;
        public static final int WindowButton = 13;

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}