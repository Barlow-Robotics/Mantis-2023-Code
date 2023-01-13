// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        
        public static int driveMotorLeftLeaderID = 4;
        public static int driveMotorLeftFollowerID = 5;
        public static int driveMotorRightLeaderID = 6;
        public static int driveMotorRightFollowerID = 7;
        
        //EP confirm variables with kinahan; move any outside of class if universal
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double driveSpeed = 0.5; //EP percentOutput?

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;
        
        public static final double driveGearRatio = 10.71;
        public static final double countsPerRevolution = 2048.0 * driveGearRatio;
        public static final double wheelDiameter = 6.0 * inchesToMeters;
        public static final double metersPerRevolution = wheelDiameter * Math.PI ;
        public static final double metersPerCount = metersPerRevolution / countsPerRevolution;
    }

    public static final class ArmConstants {
		
        public static final int armTelescopeMotorID = 0;  //EP change
        public static final int armRotateMotorID = 0;     //EP change 

        public static final int armAngleTolerance = 2; //Need to change this
        public static final int armLengthTolerance = 2; //Need to change this
        
        //add values when we figure out angles
        public static final double topRowArmAngle = 0;
        public static final double middleRowArmAngle = 0;
        public static final double bottomRowArmAngle = 0;
        public static final double restingArmAngle = 90;

        //add values when we figure out lengths
        public static final double topRowArmLength = 0;
        public static final double middleRowArmLength = 0;
        public static final double bottomRowArmLength = 0;
        public static final double restingArmLength = 0;

    }

    public static final class ClawConstants {

        public static final int clawMotorID = 0;  //EP change

        //add values when we figure out angles
        public static final double topRowClawAngle = 0;
        public static final double middleRowClawAngle = 0;
        public static final double bottomRowClawAngle = 0;
        public static final double restingClawAngle = 0;
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {

		public static final int cameraLightID = 0;
    }

    public static final class PathConstants {
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

        public static final double forwardAxisAttenuation = -0.5 ;
        public static final double lateralAxisAttenuation = 0.5 ;
        public static final double yawAxisAttenuation = 0.5 ;
    }
    
    public final class RadioMasterConstants {

        //EP need to confirm that these are correct for RadioMasterTX12
        public static final int leftGimbalX = 3;
        public static final int rightGimbalX = 0;
        public static final int rightGimbalY = 1;

        public static final int SB3Axis = 6 ;
        public static final int SFAxis = 4 ;
        public static final int SEAxis = 5 ;
        public static final int SHMomentary = 4 ;
        public static final int SCButton = 1 ;

        public static final double forwardAxisAttenuation = 1.0 ;
        public static final double lateralAxisAttenuation = 1.0 ;
        public static final double yawAxisAttenuation = 0.6 ;
    }
}
