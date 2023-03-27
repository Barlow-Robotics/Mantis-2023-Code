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

        public static final double MaxSpeed = 3.8; // meters per second
        // public static final double maxAngularSpeed = 2 * Math.PI; // one rotation per
        // second
        public static final double TimeToReachVelocity = 0.2;

        // public static final double MaxVelocityChange = MaxSpeed * (
        // TimeToReachVelocity / 50 ); // percentage of Acceleration
        public static final double MaxVelocityChange = MaxSpeed * 0.4; // percentage of Acceleration

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

        public static final double ClosedVoltageRampingConstant = 0.25;
        public static final double ManualVoltageRampingConstant = 0.25;
        // public static final double kF = 20.0; //0.048 original
        // public static final double kP = 0.7; //0.005 original
        // public static final double kI = 0.001; //0.0001 original
        public static final double kF = 0.048;
        public static final double kP = 0.005;
        public static final double kI = 0.0001; // 0.0001 original
        public static final double kD = 0.0;
        public static final int PID_id = 0;

        public static final double BalanceTolerance = 5.5;
        public static final double AutoBalanceSpeed = 0.15;
        public static final double AutoBalanceSlowerSpeed = 0.075;
        public static final double AutoBalanceKP = 0.01071428571;
        public static final double BalanceMaxAngle = 14;
        public static final double BalanceMinAngle = 2.5;

        // public static final double DefaultAutoVelocity = 3.0;
        public static final double DefaultAutoVelocity = 1.0;
        public static final double DefaultAutoAccel = 4.0;

        public static final double AutoAlignkP = 0.003;
        public static final double AutoAlignkI = 0.0;
        public static final double AutoAlignkD = 0.0;
    }

    public static final class ArmConstants {
        public static final double RotateGearRatio = 100;
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

        public static final double armRotateSpeed = 70; // Degrees per second
        public static final double armRotateAccelerationTime = 0.25;
        public static final double armExtendSpeed = 25; // Inches per second
        public static final double armExtendAccelerationTime = 0.25; // Need to change (seconds for total motion)

        public static final double ExtentionCalibrationVelocity = -1.0;
        public static final double RotationCalibrationVelocity = 10; // Degrees per second

        public static final int ArmExtendMotorID = 12;
        public static final int LeftArmMotorID = 11;
        public static final int RightArmMotorID = 10;

        public static final int AccelerationSmoothing = 2;

        public static final double ArmAngleTolerance = 2.0;
        public static final double ArmLengthTolerance = 1.0;

        // wpk need to add constants for slow and fast velocities for moving the arm.

        /* Automatic Arm Positioning Constants */

        public static final double RestingArmAngle = 0.0;
        public static final double RestingArmLength = 0.25;

        public static final double FloorArmAngle = 35.0;
        public static final double FloorArmLength = 11.0; // inches

        public static final double PlayerStationArmAngle = 105.0;
        public static final double PlayerStationArmLength = 0.0;

        public static final double TopArmAngle = 115.0;
        public static final double TopArmLength = 26.75; // inches

        public static final double MiddleArmAngle = 105.0;
        public static final double MiddleArmLength = 10.0; // inches

        public static final double BottomArmAngle = 60.0;
        public static final double BottomArmLength = 11.8; // inches

        public static final double AvoidChassisArmAngle = 30.0;
        public static final double AvoidChassisArmLength = 0.0;

        public static final double ArmMinAngle = 0.0;
        public static final double ArmMaxAngle = 120.0;
        public static final double ArmMinLength = 0.0;
        public static final double ArmMaxLength = 26.0; // need to confirm (inches)

        public static final double RotateVel = 75;
        public static final double RotateAccel = 0.15;
        public static final double AngleMultiplier = 1.0;
        public static final double AngleVel = 0.5;
        public static final double AngleAcceleration = RotateVel * 4.0;
        // public static final double AngleMultiplier = 0.5;

        public static final double ExtendVel = 26.0;
        public static final double ExtendAccel = 0.1;
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

        public static final double ffRetracted = 0.13; // Need to re-test to find this after claw is attached
        public static final double ffExtracted = 0.36; // Need to re-test to find this after claw is attached

        public static final double extendFF = -0.06;
        // public static final int kMeasuredPosHorizontal = 840; // ALH - this is
        // supposed to be position measured when arm is horizontal, need to determine
        // what that is.
    }

    public static final class ClawConstants {

        public static final int ClawMotorID = 15;
        public static final int OpenSolenoidID = 3;
        public static final int CloseSolenoidID = 2;

        // public static final double RotateGearRatio = 32.861;
        public static final double RotateGearRatio = 31.00;

        public static final double RevolutionsPerDegree = 1.0 / 360.0;

        public static final double ClawClosedVoltageRampingConstant = 0.0;
        public static final double ClawManualVoltageRampingConstant = 0.0;
        public static final double ClawKF = 0.048;
        public static final double ClawKP = 0.5;
        public static final double ClawKI = 0.0000;
        public static final double ClawKD = 0.1;
        public static final int ClawPID_id = 0;
        public static final double ff = 0.0;

        public static final double CountsPerClawDegree = TalonFXEncoderResolution * RevolutionsPerDegree
                * RotateGearRatio; // Need to change

        public static final double DegreesPerSecToCountsPer100MSec = CountsPerClawDegree * SecondsTo100MSec;

        public static final double ClawAngleTolerance = 1.0;

        public static final double ClawSpeed = 100.0; // Needs tuning
        public static final int DistanceSensorID = 50; // Need to change
        public static final int AccelerationSmoothing = 2;

        public static final double InchesForAutoClosing = 6;
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

        public static final double DriveToGamePieceSpeed = 0.5; // meters per second
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

        public static final int ButtonD = 2;
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
        // public static final int BackButton = 7;
        public static final int StartButton = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;
        public static final int WindowButton = 7;

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}