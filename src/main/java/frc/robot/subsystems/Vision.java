// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  DigitalOutput cameraLight;

  public Vision() {
    cameraLight = new DigitalOutput(Constants.VisionConstants.cameraLightID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public void turnOnVisionLight() {
    //turn the green LEDs on and off.
    //The LEDs will be controlled by a digital output from the RoboRio.
    //true = on
    cameraLight.set(true);
  }

  public void turnOffVisionLight() {
    //turn the green LEDs on and off.
    //The LEDs will be controlled by a digital output from the RoboRio.
    //false = off
    cameraLight.set(false);
  
  }

  public boolean aprilTagIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_detected").getBoolean(false);
  }

  public double aprilTagDistanceFromCenter() {
    //returns the number of pixels from the center of the screen to the center of the vision target. 
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_distance_from_center").getDouble(0.0);
  }


  public boolean coneIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cone_detected").getBoolean(false);
  }

  public double coneDistanceFromCenter() {
    //tell how many pixels the cone is from the center of the screen.
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cone_distance_from_center").getDouble(0.0);
  }

  public boolean cubeIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cube_detected").getBoolean(false);
  }

  public double cubeDistanceFromCenter() {
    //tell how many pixels the cube is from the center of the screen.
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cube_distance_from_center").getDouble(0.0);
  }

  public boolean poleIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/pole_detected").getBoolean(false);
  }

  public double poleDistanceFromCenter() {
    //tell how many pixels the pole is from the center of the screen.
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/pole_distance_from_center").getDouble(0.0);
  }

  public double bbConeHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_cone_height").getDouble(0.0);
  }

  public double bbConeWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_cone_width").getDouble(0.0);
  }

  public double bbCubeHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_cube_height").getDouble(0.0);
  }

  public double bbCubeWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_cube_width").getDouble(0.0);
  }

  public double bbPoleHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_pole_height").getDouble(0.0);
  }

  public double bbPoleWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_pole_width").getDouble(0.0);
  }
  
  public double bbAprilTagHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_april_tag_height").getDouble(0.0);
  }

  public double bbAprilTagWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/target_bb_april_tag_width").getDouble(0.0);
  }


}

