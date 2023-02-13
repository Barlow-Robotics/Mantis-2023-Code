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

    public static final double inchesToMeters = 0.0254;

    public static final class DriveConstants {
        public static final int driveMotorLeftLeaderID = 4;
        public static final int driveMotorLeftFollowerID = 5;
        public static final int driveMotorRightLeaderID = 6;
        public static final int driveMotorRightFollowerID = 7;

        //confirm variables with kinahan; move any outside of class if universal
        public static final double maxSpeed = 4.0; // meters per second
        // public static final double maxAngularSpeed = 2 * Math.PI; // one rotation per second

        public static final double trackWidth = 0.381 * 2; // meters      NEED TO UPDATE

        public static final int encoderResolution = 2048; //2048 for talonfx, 4096 for talonsrx
        public static final double driveGearRatio = 9.8;
        public static final double countsPerWheelRevolution = encoderResolution * driveGearRatio;
        
        public static final double wheelDiameter = 6.0 * inchesToMeters;
        public static final double metersPerRevolution = wheelDiameter * Math.PI;
        public static final double RevolutionsPerMeter = 1.0 / metersPerRevolution ;

        public static final double MetersPerSecondToCountsPerSecond =  RevolutionsPerMeter * countsPerWheelRevolution ;
        public static final double MaxSpeedCountsPer100MSec 
            = maxSpeed *MetersPerSecondToCountsPerSecond / 10.0 ; 

        // public static final double metersPerCount = metersPerRevolution / countsPerWheelRevolution;
        // public static final double CountsPerMeterPerSecond = 1 / metersPerCount;
        public static final double CorrectionRotationSpeed = 0;

        public static final double closedVoltageRampingConstant = 0.0;
        public static final double manualVoltageRampingConstant = 0.0;
        public static final double kF = 0.048;
        public static final double kP = 0.005;
        public static final double kI = 0.0001;
        public static final double kD = 0.0;
        public static final int PID_id = 0;
    }

    public static final class ArmConstants {
        public static final double armRotateSpeed = 0; // Need to change this
        public static final double armExtendSpeed = 0; // Need to change this

        public static final int armExtendMotorID = 0; // change
        public static final int armLeaderMotorID = 0;
        public static final int armFollowMotorID = 0;
        
		public static final double rotateGearRatio = 40;
        public static final double extendGearRatio = 0;

        public static final int AccelerationSmoothing = 2 ;

        public static final int armAngleTolerance = 2; // Need to change this
        public static final int armLengthTolerance = 2; // Need to change this

        // add values when we figure out lengths

        public static final double countsPerRevolution = 0;
        public static final double metersPerRevolution = 0;
        public static final double revolutionsPerDegree = 1.0 / metersPerRevolution ;

        public static final double CountsPerArmDegree = countsPerRevolution * revolutionsPerDegree;
        public static final double CountsPerArmInch = 0;

        // wpk need to add constants for slow and fast velocities for moving the arm.

        // wpk need to add constants for arm angles and extensions for:
        // Home (example. Real values may be different)
        public static final double HomeArmAngle = 0.0 ;
        public static final double HomeArmExtension = 0.0 ;
        // Floor pickup
        // Substation Pickup
        // Grid level 1
        // Grid level 2
        // Grid level 3

        // wpk will probably need to add constants for minimum angle before retracting arm. This is required to avoid
        // crashing the claw into the chassis when going to home from a position where the claw is above the chassis.

    }

    public static final class ClawConstants {
        public static final int clawMotorID = 0; // change

        public static final double CountsPerClawDegree = 0; // Need to change

        public static final int ClawAngleTolerance = 0; // Need to change

        public static final double clawSpeed = 0; // Need to change
    }

    public static final class UnderGlowConstants {
        public static final SerialPort.Port port = SerialPort.Port.kUSB1;
        public static final int BlueAliance = 1;
        public static final int RedAliance = 2;
        public static final int NeonGreen = 3;
    }

    public static final class VisionConstants {
        public static final int cameraLightID = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.25; 
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 10*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond / 0.5;
    }

    public final class LogitechDualActionConstants {
        public static final int leftJoystickX = 0;
        public static final int leftJoystickY = 1;
        public static final int rightJoystickX = 2;
        public static final int rightJoystickY = 3;

        public static final int leftTrigger = 7;
        public static final int rightTrigger = 8;
        public static final int leftBumper = 5;
        public static final int rightBumper = 6;
        public static final int buttonA = 2;
        public static final int buttonB = 3;
        public static final int buttonX = 1;
        public static final int buttonY = 4;
        public static final int backButton = 9;
        public static final int startButton = 10;
        public static final int leftJoystick = 11;
        public static final int rightJoystick = 12;

        public static final double forwardAxisAttenuation = -0.5;
        public static final double lateralAxisAttenuation = 0.5;
        public static final double yawAxisAttenuation = 0.5;
    }

    public final class RadioMasterConstants {

        // EP need to confirm that these are correct for RadioMasterTX12
        public static final int leftGimbalX = 0;
        public static final int leftGimbalY = 1;
        public static final int rightGimbalX = 3;
        public static final int rightGimbalY = 2;

        public static final int SB3Axis = 6;
        public static final int SFAxis = 4;
        public static final int SEAxis = 5;
        public static final int SHMomentary = 4;
        public static final int SCButton = 1;

        public static final double forwardAxisAttenuation = 1.0;
        public static final double lateralAxisAttenuation = 1.0;
        public static final double yawAxisAttenuation = 0.6;
    }

    public final static double kNeutralDeadband = 0.001;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * 	                                    			  kP   kI    kD     kF             Iz    PeakOut */
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
	
	public final static int kPrimaryPIDSlot = 0; // any slot [0,3]
}