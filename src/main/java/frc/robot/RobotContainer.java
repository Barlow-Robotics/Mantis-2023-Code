// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

// import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignWithAprilTags;
import frc.robot.commands.AlignWithGamePiece;
import frc.robot.commands.AlignWithPole;
import frc.robot.commands.MoveArm;
// import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private final Drive driveSub = new Drive();
    private final Arm armSub = new Arm();
    private final Claw clawSub = new Claw(armSub);
    private final Vision visionSub = new Vision();
    // private final Underglow underglowSub = new Underglow();
    // private final Vision visionSub = new Vision();

    // private final TurnOffUnderglow turnOffUnderGlowCom = new
    // TurnOffUnderglow(underglowSub);
    // private final TurnOnUnderglow turnOnUnderGlowCom = new
    // TurnOnUnderglow(underglowSub);

    Joystick driverController; // Joystick 1
    Joystick operatorController; // Joystick 2

    PIDController pid;

    private int xAxis;
    private int yawAxis;
    private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;

    private int angle;
    private int extension;

    private Trigger moveToTopButton;
    private Trigger moveToMiddleButton;
    private Trigger moveToBottomButton;
    private Trigger moveToRestingPositionButton;
    private Trigger moveToPlayerStationButton;
    private Trigger alignWithAprilTagsButton;
    private Trigger alignWithGamePieceButton;
    private Trigger alignWithPoleButton;
    private Trigger moveToFloorButton;

    private Command moveToBottom;
    private Command moveToMiddle;
    private Command moveToResting;
    private Command moveToFloor;

    // buttons

    public RobotContainer() {
        configureButtonBindings();

        driveSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> {

                            boolean autoSteer = alignWithGamePieceButton.getAsBoolean();

                            double x = -driverController.getRawAxis(xAxis);
                            if (Math.abs(x) < 0.01) {
                                x = 0.0;
                            }
                            double yaw = -driverController.getRawAxis(yawAxis);
                            if (Math.abs(yaw) < 0.01) {
                                yaw = 0.0;
                            }
                            // fancy exponential formulas to shape the controller inputs to be flat when
                            // only pressed a little, and ramp up as stick pushed more.
                            double speed = 0.0;
                            if (x != 0) {
                                speed = (Math.abs(x) / x) * (Math
                                        .exp(-400.0 * Math.pow(x / 3.0, 4.0)))
                                        + (-Math.abs(x) / x);
                            }
                            double turn = -yaw;

                            // double turn = 0.0;
                            // if (yaw != 0) {
                            // turn = (Math.abs(yaw) / yaw) * (Math.exp(-400.0 *
                            // Math.pow(yaw / 3.0, 4.0)))
                            // + (-Math.abs(yaw) / yaw);
                            // }

                            // The turn input results in really quick movement of the bot, so let's reduce
                            // the turn input and make it even less if we are going faster. This is a simple
                            // y = mx + b equation to adjust the turn input based on the speed.

                            // turn = turn * (-0.4 * Math.abs(speed) + 0.5);

                            if (!autoSteer || !clawSub.clawIsOpen()) {
                                yaw = -turn;

                                yawMultiplier = (float) (0.6 + Math.abs(speed) * 0.2f);

                                yaw = (Math.abs(yaw) / yaw) * (yaw * yaw) * yawMultiplier;
                                if (Math.abs(yaw) < 0.05f) {
                                    yaw = 0.0f;
                                }
                                lastAutoSteer = false;
                            } else {
                                if (!lastAutoSteer) {
                                    pid.reset();
                                }
                                yaw = -pid.calculate(visionSub.gamePieceDistanceFromCenter());
                                lastAutoSteer = true;
                            }

                            driveSub.drive(-speed, -turn * 0.4, false);

                        },
                        driveSub));

        armSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> {
                            double x = operatorController.getRawAxis(angle);
                            if (Math.abs(angle) < 0.01) {
                                x = 0.0;
                            }

                            double yaw = operatorController.getRawAxis(extension);
                            if (Math.abs(extension) < 0.01) {
                                yaw = 0.0;
                            }

                            armSub.startRotatingAtVelocty(angle);
                            armSub.startExtendingAtVelocty(extension);
                        },
                        armSub));

    }

    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorController = new Joystick(2);
        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);

        xAxis = Constants.LogitechDualActionConstants.leftJoystickY;
        yawAxis = Constants.LogitechDualActionConstants.rightJoystickX;

        angle = Constants.LogitechDualActionConstants.rightJoystickY; // need to check with unity sim
        extension = Constants.LogitechDualActionConstants.leftJoystickY; // need to check with unity sim

        /* * * * * * ARM BUTTONS * * * * * */

        moveToBottom = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration),
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.BottomArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.BottomArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration));

        moveToBottomButton = new JoystickButton(operatorController, 4);
        moveToBottomButton.onTrue(moveToBottom);

        if (armSub.state().equalsIgnoreCase("Floor") || armSub.state().equalsIgnoreCase("Bottom")) {
            moveToResting = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingFromFloorArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            0, // *** Need to change (says "armContronl.Extention" in sim) - Angela
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration));
        } else {
            moveToResting = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            0, // *** Need to change (says "armContronl.Rotation()" in sim) - Angela
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration));
        }

        moveToRestingPositionButton = new JoystickButton(operatorController, 5);
        moveToRestingPositionButton.onTrue(moveToResting);

        moveToFloor = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration), 
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.FloorArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.FloorArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration));

        moveToTopButton = new JoystickButton(operatorController, 2);
        moveToTopButton.onTrue(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.TopArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.TopArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration)); 

        moveToFloor = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration),
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.FloorArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAcceleration,
                        Constants.ArmConstants.FloorArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAcceleration));

        moveToFloorButton = new JoystickButton(operatorController, 4);
        moveToFloorButton.onTrue(moveToFloor);

        if (armSub.state().equalsIgnoreCase("Floor") || armSub.state().equalsIgnoreCase("Bottom")) {
            moveToMiddle = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleFromBottomArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.MiddleFromBottomArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.MiddleArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration));
        } else { // *** Need to fix: right now these are the same b/c that's how it is in the sim - Angela
            moveToMiddle = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleFromBottomArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.MiddleFromBottomArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAcceleration,
                            Constants.ArmConstants.MiddleArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAcceleration));
        }

        moveToMiddleButton = new JoystickButton(operatorController, 3);
        moveToMiddleButton.onTrue(moveToMiddle);

        moveToPlayerStationButton = new JoystickButton(operatorController, 5);
        moveToPlayerStationButton.onTrue(
                new MoveArm(armSub, Constants.ArmConstants.PlayerStationArmAngle, Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armExtendAcceleration, Constants.ArmConstants.PlayerStationArmLength,
                        Constants.ArmConstants.armExtendSpeed, Constants.ArmConstants.armExtendAcceleration));

        /* * * * * * VISION BUTTONS * * * * * */

        alignWithAprilTagsButton = new JoystickButton(operatorController, 6);
        moveToRestingPositionButton.onTrue(new AlignWithAprilTags(visionSub, driveSub));

        alignWithGamePieceButton = new JoystickButton(operatorController, 6);
        moveToRestingPositionButton.onTrue(new AlignWithGamePiece(visionSub, driveSub));

        alignWithPoleButton = new JoystickButton(operatorController, 6);
        moveToRestingPositionButton.onTrue(new AlignWithPole(visionSub, driveSub));
    }

    public Command getAutonomousCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("event1", new PrintCommand("Passed first leg"));
        eventMap.put("event2", new PrintCommand("half way there"));
        eventMap.put("event3", new PrintCommand("almost, i swear"));
        eventMap.put("event4", new PrintCommand("arrived at detination"));

        PathPlannerTrajectory traj = PathPlanner.loadPath("Test", new PathConstraints(1, 4));

        RamseteController controller = new RamseteController();

        Command ic = new InstantCommand(() -> {
            // driveSub.resetEncoders();
            driveSub.resetOdometry(traj.getInitialPose());
        });

        Command pathFollowingCommand = new PPRamseteCommand(
                traj,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub);

        Command followPathWithEvents = new FollowPathWithEvents(pathFollowingCommand, traj.getMarkers(),
                eventMap);

        // return new SequentialCommandGroup(ic, pathFollowingCommand);
        return new SequentialCommandGroup(ic, followPathWithEvents);
    }
}