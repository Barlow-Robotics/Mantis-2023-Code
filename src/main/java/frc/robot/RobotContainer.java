// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.RamseteController ;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final Drive driveSub = new Drive();
    private final Underglow underglowSub = new Underglow();
    private final Vision visionSub = new Vision();

    private final TurnOffUnderglow turnOffUnderGlowCom = new TurnOffUnderglow(underglowSub);
    private final TurnOnUnderglow turnOnUnderGlowCom = new TurnOnUnderglow(underglowSub);

    Joystick driverController; // Joystick 1
    Joystick operatorController; // Joystick 2

    // buttons


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings'
        configureButtonBindings();
        // loadTrajectories();
        // createAutonomousCommands();

        driveSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand( // new instance
                        () -> {
                            double x = -driverController.getRawAxis(Constants.LogitechDualActionConstants.leftJoystickY);
                            double yaw = -driverController.getRawAxis(Constants.LogitechDualActionConstants.rightJoystickX);
                            // fancy exponential formulas to shape the controller inputs to be flat when
                            // only
                            // pressed a little, and ramp up as stick pushed more.
                            double speed = 0.0;
                            if (x != 0) {
                                speed = (Math.abs(x) / x) * (Math.exp(-400.0 * Math.pow(x / 3.0, 4.0)))
                                        + (-Math.abs(x) / x);
                            }
                            double turn = -yaw;
                            // double turn = 0.0;
                            // if (yaw != 0) {
                            // turn = (Math.abs(yaw) / yaw) * (Math.exp(-400.0 * Math.pow(yaw / 3.0, 4.0)))
                            // + (-Math.abs(yaw) / yaw);
                            // }
                            // The turn input results in really quick movement of the bot, so
                            // let's reduce the turn input and make it even less if we are going faster
                            // This is a simple y = mx + b equation to adjust the turn input based on the
                            // speed.
                            // turn = turn * (-0.4 * Math.abs(speed) + 0.5);

                            driveSub.drive(-speed, -turn * 0.4, false);
                        },
                        driveSub));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorController = new Joystick(2);

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);

        // Add code to reverse axes 2 and 3 if driverController is the radiomaster


        
        // button = new JoystickButton(controller, constant);

        // button.whenPressed/whileHeld(command);
    }   

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
   
        public Command getAutonomousCommand() {
        // This is just an example event map. It would be better to have a constant, global event map in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("FirstBase", new PrintCommand("Passed first leg"));
        // eventMap.put("half way", new PrintCommand("half way there"));
        // eventMap.put("done", new PrintCommand("arrived at detination"));

        // This will load the file "Example Path.path" and generate it with a max velocity of 2 m/s and a max acceleration of 4 m/s^2
        PathPlannerTrajectory traj = PathPlanner.loadPath("Test", new PathConstraints(2, 4));        

        Command ic = new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            driveSub.resetEncoders();
            driveSub.resetOdometry(traj.getInitialPose());
        });

        RamseteController controller = new RamseteController();

        Command pathFollowingCommand = new PPRamseteCommand(
            traj, 
            driveSub::getPose, 
            controller, 
            new DifferentialDriveKinematics(0.75),
            driveSub::setSpeeds, 
            true, 
            driveSub
            ); 
        
        return new SequentialCommandGroup(ic, pathFollowingCommand);
    }
}