// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    //HashMap<String, PathPlannerTrajectory> trajectories;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        // loadTrajectories();
        // createAutonomousCommands();

        driveSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand( // new instance
                        () -> {
                            double x = -driverController.getRawAxis(Constants.LogitechDualActionConstants.leftJoystickY);
                            double yaw = driverController.getRawAxis(Constants.LogitechDualActionConstants.rightJoystickX);
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

    // private void loadTrajectory(String name, double maxVel, double maxAccel) {
    //     PathPlannerTrajectory theTrajectory = PathPlanner.loadPath(name, maxVel, maxAccel);

    //     for (var s : theTrajectory.getStates()) {
    //         PathPlannerState pps = (PathPlannerState) s;
    //     }

    //     trajectories.put(name, theTrajectory);
    // }

    // private void loadTrajectories() {
    //     trajectories = new HashMap<String, PathPlannerTrajectory>();
    //     double maxVel = AutoConstants.kMaxSpeedMetersPerSecond;
    //     double maxAccel = AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    //     loadTrajectory(name, maxVel, maxAccel);

    //     // // wpk delete this code block after testing complete
    //     // PathPlannerTrajectory temp = PathPlanner.loadPath("Two_Ball_Low_Goal",
    //     // maxVel, maxAccel) ;
    //     // try {
    //     // String fileName = Filesystem.getDeployDirectory().getPath() +
    //     // "\\path_samples.csv" ;
    //     // FileWriter myWriter = new FileWriter(fileName);
    //     // myWriter.write("Sample#,Rotation\n") ;
    //     // for ( int i = 0; i < temp.getStates().size(); i++) {
    //     // PathPlannerState pps = (PathPlannerState) temp.getStates().get(i) ;
    //     // String s = String.format("%d, %7.4f%n", i,
    //     // pps.holonomicRotation.getDegrees()) ;
    //     // myWriter.write(s);
    //     // }
    //     // myWriter.close();
    //     // } catch ( Exception ex) {
    //     // System.out.println("failed to load path") ;
    //     // }

    // }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (driverController == null) {
            System.out.println("Null driver controller, using joystick 1");
            driverController = new Joystick(1);
        }

        if (operatorController == null) {
            System.out.println("Null operator controller, using joystick 2");
            operatorController = new Joystick(2);
        }

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);
        // boolean controllerFound = false;

        // button = new JoystickButton(controller, constant);

        // button.whenPressed/whileHeld(command);
    }

    // public Command getAutonomousCommand() {
    //     String autoCommandName = NetworkTableInstance.getDefault().getEntry("autonomous/auto_command_name")
    //             .getString("Simple Shoot and Back Up");
    //     System.out.println("Using autonomous commad " + autoCommandName);
    //     AutonomousCommandHolder commandHolder = autonomousCommands.get(autoCommandName);
    //     if (commandHolder != null) {
    //         m_robotDrive.resetOdometry(commandHolder.initialPose);
    //         return commandHolder.autonomousCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    //     } else {
    //         return new PrintCommand("Invalid autonomous command name")
    //                 .andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    //     }
    // }

    // private void createAutonomousCommands() {
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    // // An ExampleCommand will run in autonomous

    // return autonomousCommand;
    // }
}