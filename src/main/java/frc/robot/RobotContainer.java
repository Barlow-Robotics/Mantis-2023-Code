// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.*;
import frc.robot.subsystems.*;

// import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.RamseteController;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

    private final Drive driveSub = new Drive();
    private final Arm armSub = new Arm();
    // private final Underglow underglowSub = new Underglow();
    // private final Vision visionSub = new Vision();

    // private final TurnOffUnderglow turnOffUnderGlowCom = new
    // TurnOffUnderglow(underglowSub);
    // private final TurnOnUnderglow turnOnUnderGlowCom = new
    // TurnOnUnderglow(underglowSub);

    Joystick driverController; // Joystick 1
    Joystick operatorController; // Joystick 2

    private int xAxis;
    private int yawAxis;

    private int angle;
    private int extention;

    // buttons

    public RobotContainer() {
        configureButtonBindings();

        driveSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand( // new instance
                        () -> {
                            double x = -driverController.getRawAxis(xAxis);
                            if ( Math.abs(x) < 0.01) {
                                x = 0.0 ;
                            }
                            double yaw = -driverController.getRawAxis(yawAxis);
                            if ( Math.abs(yaw) < 0.01) {
                                yaw = 0.0 ;
                            }
                            // double angle = -operatorController
                            // .getRawAxis(xAxis);
                            // double extention = -operatorController
                            // .getRawAxis(yawAxis);
                            // fancy exponential formulas to shape the controller inputs to be flat when
                            // only pressed a little, and ramp up as stick pushed more.
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

                           // driveSub.setSpeeds( -2.0, -2.0) ;
                        },
                        driveSub));

        // armSub.setDefaultCommand(
        //         new RunCommand( // new instance
        //                 () -> {
        //                     double x = m_operatorController.getRawAxis(Constants.Logitech_Dual_Action.Right_Stick_X);
                                  
        //                     armSub.armRotate(x);
        //                     double yaw = m_operatorController.getRawAxis(Constants.LogitechDual_Dual_Action.Right_Stick_yaw);

        //                     armSub.armRotate(yaw);

                                    
        //                 },
        //                 armSub));

        // Do we want the claw to open/close w/ a button?
        // climbButton = new JoystickButton(operatorController,
        // Constants.LogitechDualActionConstants.buttonA);

        // Button.whenPressed(climbCommand);

    }

    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorController = new Joystick(2);
        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);

        // if (controllerType == "RM TX16S Joystick") {
        // xAxis = Constants.RadioMasterConstants.leftGimbalY;
        // yawAxis = Constants.RadioMasterConstants.rightGimbalX;
        // } else {
        xAxis = Constants.LogitechDualActionConstants.leftJoystickY;
        yawAxis = Constants.LogitechDualActionConstants.rightJoystickX;

        // }

        angle = Constants.LogitechDualActionConstants.rightJoystickY;
        extention = Constants.LogitechDualActionConstants.leftJoystickY;

        // button = new JoystickButton(controller, constant);

        // button.whenPressed/whileHeld(command);
    }

    public Command getAutonomousCommand() {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("FirstBase", new PrintCommand("Passed first leg"));
        // eventMap.put("half way", new PrintCommand("half way there"));
        // eventMap.put("done", new PrintCommand("arrived at detination"));

        PathPlannerTrajectory traj = PathPlanner.loadPath("Test", new PathConstraints(2, 4));

        Command ic = new InstantCommand(() -> {
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
                driveSub);

        return new SequentialCommandGroup(ic, pathFollowingCommand);
    }
}