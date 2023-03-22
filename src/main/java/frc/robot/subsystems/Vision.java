// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.HashMap;
import java.util.Map;

// import com.fasterxml.jackson.core.json.*;
import com.fasterxml.jackson.core.type.TypeReference;

// import org.json.JSONObject;

// import com.fasterxml.jackson.core.util.*;
// import org.json.JSONObject;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

public class Vision extends SubsystemBase implements Sendable {
    /** Creates a new Vision. */

    DigitalOutput cameraLight;

    boolean gamePieceDetected;
    double gamePieceDistanceFromCenter;
    double gamePieceHeight;
    double gamePieceWidth;

    private DatagramChannel visionChannel = null;
    ByteBuffer buffer = ByteBuffer.allocate(1024);

    public Vision() {
        cameraLight = new DigitalOutput(Constants.VisionConstants.CameraLightID);
        try {
            visionChannel = DatagramChannel.open();
            InetSocketAddress sAddr = new InetSocketAddress(5800);
            visionChannel.bind(sAddr);
            visionChannel.configureBlocking(false);
        } catch (Exception ex) {
            int wpk = 1 ;
        }
    }

    @Override
    public void periodic() {
        try {
            boolean done = false;
            String message = "";
            while (!done) {
                SocketAddress sender = visionChannel.receive(buffer);
                buffer.flip();
                int limits = buffer.limit();
                if (limits > 0) {
                    byte bytes[] = new byte[limits];
                    buffer.get(bytes, 0, limits);
                    message = new String(bytes);
                } else {
                    done = true;
                }
                buffer.clear();
            }

            if (message.length() > 0) {
                Map<String, String> myMap = new HashMap<String, String>();

                ObjectMapper objectMapper = new ObjectMapper();
                myMap = objectMapper.readValue(message, new TypeReference<HashMap<String, String>>() {
                });
                this.gamePieceDetected = Boolean.parseBoolean(myMap.get("detected"));
                this.gamePieceDistanceFromCenter = Double.parseDouble(myMap.get("distance_from_center"));
                this.gamePieceHeight = Double.parseDouble(myMap.get("bb_height"));
                this.gamePieceWidth = Double.parseDouble(myMap.get("bb_width"));
            }

            // var Vision_Info = new JSONObject(received);

            // double aprilTagDistanceFromCenter =
            // Vision_Info.get(april_tag_distance_from_center);
        } catch (Exception ex) {
            System.out.println("Exception reading data");
        }
    }

    public void turnOnVisionLight() {
        // turn the green LEDs on and off.
        // The LEDs will be controlled by a digital output from the RoboRio.
        // true = on
        cameraLight.set(true);
    }

    public void turnOffVisionLight() {
        // turn the green LEDs on and off.
        // The LEDs will be controlled by a digital output from the RoboRio.
        // false = off
        cameraLight.set(false);

    }

    public boolean aprilTagIsVisible() {
        // The data for this will come from the Jetson Nano via network tables.
        // return
        // NetworkTableInstance.getDefault().getEntry("vision/april_tag_detected").getBoolean(false);
        // return Vision_Info.get("vision/april_tag_detected");
        return false;
    }

    public double aprilTagDistanceFromCenter() {
        // returns the number of pixels from the center of the screen to the center of
        // the vision target.
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/april_tag_distance_from_center").getDouble(0.0);
    }

    public boolean gamePieceIsVisible() {
        return this.gamePieceDetected;
    }

    public double gamePieceDistanceFromCenter() {
        // tell how many pixels the gamePiece is from the center of the screen.
        return this.gamePieceDistanceFromCenter;
    }

    public boolean poleIsVisible() {
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/pole_detected").getBoolean(false);
    }

    public double poleDistanceFromCenter() {
        // tell how many pixels the pole is from the center of the screen.
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/pole_distance_from_center").getDouble(0.0);
    }

    public double bbGamePieceHeight() {
        return this.gamePieceHeight;
    }

    public double bbGamePieceWidth() {
        return this.gamePieceWidth;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Vision Subsystem");
        builder.addBooleanProperty("Game piece detected", this::gamePieceIsVisible, null);
        builder.addDoubleProperty("Game piece distance from center", this::gamePieceDistanceFromCenter, null);
        builder.addDoubleProperty("Game piece height", this::bbGamePieceHeight, null);
        builder.addDoubleProperty("Game piece width", this::bbGamePieceWidth, null);
    }
}