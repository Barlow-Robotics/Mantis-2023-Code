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

    public static final class DriveConstants {
        
        //EP confirm variables with kinahan; move any outside of class if universal
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double driveSpeed = 0.5;

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;
        
        public static final double driveGearRatio = 10.71;
        public static final double Counts_Per_Revolution = 2048.0 * driveGearRatio;
        public static final double InchesToMeters = 0.0254;
        public static final double Wheel_Diameter = 6.0 * InchesToMeters;
        public static final double Meters_Per_Revolution = Wheel_Diameter * Math.PI ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution;

        public static int driveMotorLeftLeaderID = 4;
        public static int driveMotorLeftFollowerID = 5;
        public static int driveMotorRightLeaderID = 6;
        public static int driveMotorRightFollowerID = 7;
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {
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

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }
    
    public final class RadioMasterConstants {

        //EP need to confirm that these are correct for RadioMasterTX12
        public static final int Left_Gimbal_X = 3;
        public static final int Right_Gimbal_X = 0;
        public static final int Right_Gimbal_Y = 1;

        public static final int SB3_Axis = 6 ;
        public static final int SF_Axis = 4 ;
        public static final int SE_Axis = 5 ;
        public static final int SH_Momentary = 4 ;
        public static final int SC_Button = 1 ;

        public static final double Forward_Axis_Attenuation = 1.0 ;
        public static final double Lateral_Axis_Attenuation = 1.0 ;
        public static final double Yaw_Axis_Attenuation = 0.6 ;

    }
}
