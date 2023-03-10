// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

// import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */

    DigitalOutput cameraLight;

    private DatagramSocket socket = null;
    private byte[] buf = new byte[256];

    public Vision() {
        cameraLight = new DigitalOutput(Constants.VisionConstants.CameraLightID);
        try {
            socket = new DatagramSocket(4445);
        } catch (Exception ex) {
        }
    }

    @Override
    public void periodic() {
        // DatagramPacket packet = new DatagramPacket(buf, buf.length);
        // try {
        //     socket.receive(packet);
        //     String received = new String( packet.getData(), 0, packet.getLength());

        // // var Vision_Info = new JSONObject(received);
        // } catch (Exception ex) {}
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
        // return NetworkTableInstance.getDefault().getEntry("vision/april_tag_detected").getBoolean(false);
        // return Vision_Info.get("vision/april_tag_detected");
        return false ;
    }

    public double aprilTagDistanceFromCenter() {
        // returns the number of pixels from the center of the screen to the center of
        // the vision target.
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/april_tag_distance_from_center").getDouble(0.0);
    }

    public boolean gamePieceIsVisible() {
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/game_piece_detected").getBoolean(false);
    }

    public double gamePieceDistanceFromCenter() {
        // tell how many pixels the gamePiece is from the center of the screen.
        // The data for this will come from the Jetson Nano via network tables.
        return NetworkTableInstance.getDefault().getEntry("vision/game_piece_distance_from_center").getDouble(0.0);
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
        return NetworkTableInstance.getDefault().getEntry("vision/game_piece_bb_height").getDouble(0.0);
    }

    public double bbGamePieceWidth() {
        return NetworkTableInstance.getDefault().getEntry("vision/game_piece_bb_width").getDouble(0.0);
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
