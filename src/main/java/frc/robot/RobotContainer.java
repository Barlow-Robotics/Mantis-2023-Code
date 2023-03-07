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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LogitechDualActionConstants;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AlignWithAprilTags;
import frc.robot.commands.AlignWithGamePiece;
import frc.robot.commands.AlignWithPole;
import frc.robot.commands.MoveArm;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
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
    Joystick operatorButtonController; // Joystick 2
    Joystick operatorAxisController; // Joystick 3

    PIDController pid;

    private int xAxis;
    private int yawAxis;
    private int angleAxis;
    private int extensionAxis;

    private Trigger moveToTopButton;
    private Trigger moveToMiddleButton;
    private Trigger moveToBottomButton;
    private Trigger moveToRestingPositionButton;
    private Trigger moveToPlayerStationButton;
    private Trigger alignWithAprilTagsButton;
    private Trigger alignWithGamePieceButton;
    private Trigger alignWithPoleButton;
    private Trigger moveToFloorButton;
    private Trigger toggleClawButton;
    private Trigger testingButton;

    private Command moveToBottom;
    private Command moveToMiddle;
    private Command moveToResting;
    private Command moveToFloor;

    private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;

    private final ToggleClaw toggleClaw = new ToggleClaw(clawSub);

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
                            // fancy exponential formulas to shape the controller inputs to
                            // be flat when
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

                            // The turn input results in really quick movement of the bot,
                            // so let's reduce
                            // the turn input and make it even less if we are going faster.
                            // This is a simple
                            // y = mx + b equation to adjust the turn input based on the
                            // speed.

                            // turn = turn * (-0.4 * Math.abs(speed) + 0.5);

                            if (!autoSteer || !clawSub.isOpen()) {
                                yaw = -turn;

                                yawMultiplier = (float) (0.6 + Math.abs(speed) * 0.2f);

                                yaw = (Math.abs(yaw) / yaw) * (yaw * yaw)
                                        * yawMultiplier;
                                if (Math.abs(yaw) < 0.05f) {
                                    yaw = 0.0f;
                                }
                                lastAutoSteer = false;
                            } else {
                                if (!lastAutoSteer) {
                                    pid.reset();
                                }
                                yaw = -pid.calculate(visionSub
                                        .gamePieceDistanceFromCenter());
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
                            /* Angle */
                            double currentAngle = armSub.getAngle();
                            double desiredAngle = currentAngle
                                    + (operatorButtonController.getRawAxis(angleAxis) * ArmConstants.AngleMultiplier);

                            if (desiredAngle > ArmConstants.ArmMaxAngle) {
                                desiredAngle = ArmConstants.ArmMaxAngle;
                            } else if (desiredAngle < ArmConstants.ArmMinAngle) {
                                desiredAngle = ArmConstants.ArmMinAngle;
                            }

                            armSub.setAngle(desiredAngle, ArmConstants.AngleVel, ArmConstants.AngleAcceleration);

                            /* Extension */
                            double currentLength = armSub.getLength();
                            double desiredLength = currentLength + operatorButtonController.getRawAxis(extensionAxis)
                                    * ArmConstants.LengthMultiplier;

                            if (desiredLength > ArmConstants.ArmMaxLength) {
                                desiredLength = ArmConstants.ArmMaxLength;
                            } else if (desiredLength < ArmConstants.ArmMinLength) {
                                desiredLength = ArmConstants.ArmMinLength;
                            }

                            armSub.setLength(desiredLength, ArmConstants.LengthVel, ArmConstants.LengthAccelTime);

                            if (desiredLength * Math.cos(desiredAngle) <= 0) {
                                desiredLength = currentLength;
                            }
                        },
                        armSub));
    }

    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorButtonController = new Joystick(2);
        operatorAxisController = new Joystick(3);

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);

        xAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        yawAxis = Constants.LogitechDualActionConstants.RightJoystickX;
        angleAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        extensionAxis = Constants.LogitechDualActionConstants.RightJoystickX;

        toggleClawButton = new JoystickButton(driverController, RadioMasterConstants.ButtonA);
        toggleClawButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonY);
        toggleClawButton.onTrue(toggleClaw);

        /* * * * * * ARM BUTTONS * * * * * */

        moveToBottom = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Transition),
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.BottomArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.BottomArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Bottom));

        moveToBottomButton = new JoystickButton(operatorButtonController, XboxControllerConstants.WindowButton);
        moveToBottomButton.onTrue(moveToBottom);

        if (armSub.armState == Position.Bottom || armSub.armState == Position.Floor) {
            moveToResting = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingFromFloorArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            0, // *** Need to change (says "armContronl.Extention" in sim) -
                               // Angela
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Transition),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Resting));
        } else {
            moveToResting = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            0, // *** Need to change (says "armContronl.Rotation()" in sim)
                               // - Angela
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Transition),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.RestingArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.RestingArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Resting));
        }

        moveToRestingPositionButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftTrigger);
        moveToRestingPositionButton.onTrue(moveToResting);

        moveToFloor = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Transition),
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.FloorArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.FloorArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Floor));

        moveToTopButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftStick);
        moveToTopButton.onTrue(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.TopArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.TopArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Top));

        moveToFloor = new SequentialCommandGroup(
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.AvoidChassisArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.AvoidChassisArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Transition),
                new MoveArm(
                        armSub,
                        Constants.ArmConstants.FloorArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armRotateAccelerationTime,
                        Constants.ArmConstants.FloorArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.Floor));

        moveToFloorButton = new JoystickButton(operatorButtonController, 14); // same as move to bottom?
        moveToFloorButton.onTrue(moveToFloor);

        if (armSub.armState == Position.Bottom || armSub.armState == Position.Floor) {
            moveToMiddle = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleFromBottomArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.MiddleFromBottomArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Transition),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.MiddleArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Middle));
        } else { // *** Need to fix: right now these are the same b/c that's how it is in the sim
                 // - Angela
            moveToMiddle = new SequentialCommandGroup(
                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleFromBottomArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.MiddleFromBottomArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Transition),

                    new MoveArm(
                            armSub,
                            Constants.ArmConstants.MiddleArmAngle,
                            Constants.ArmConstants.armRotateSpeed,
                            Constants.ArmConstants.armRotateAccelerationTime,
                            Constants.ArmConstants.MiddleArmLength,
                            Constants.ArmConstants.armExtendSpeed,
                            Constants.ArmConstants.armExtendAccelerationTime,
                            Position.Middle));
        }

        moveToMiddleButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftBumper);
        moveToMiddleButton.onTrue(moveToMiddle);

        moveToPlayerStationButton = new JoystickButton(operatorButtonController, XboxControllerConstants.RightStick);
        moveToPlayerStationButton.onTrue(
                new MoveArm(armSub, Constants.ArmConstants.PlayerStationArmAngle,
                        Constants.ArmConstants.armRotateSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Constants.ArmConstants.PlayerStationArmLength,
                        Constants.ArmConstants.armExtendSpeed,
                        Constants.ArmConstants.armExtendAccelerationTime,
                        Position.PlayerStation));

        testingButton = new JoystickButton(operatorAxisController, LogitechDualActionConstants.ButtonX);
        testingButton.onTrue(
                new MoveArm(armSub,
                        100,
                        70.0,
                        0.25,
                        0,
                        0,
                        0,
                        Position.Transition));

        /* * * * * * VISION BUTTONS * * * * * */

        alignWithAprilTagsButton = new JoystickButton(driverController, 6);
        alignWithAprilTagsButton.onTrue(new AlignWithAprilTags(visionSub, driveSub));

        alignWithGamePieceButton = new JoystickButton(driverController, RadioMasterConstants.ButtonD);
        alignWithGamePieceButton.onTrue(new AlignWithGamePiece(visionSub, driveSub));

        alignWithPoleButton = new JoystickButton(driverController, 8);
        alignWithPoleButton.onTrue(new AlignWithPole(visionSub, driveSub));
    }

    public Command getAutonomousCommand() {
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("event1", new PrintCommand("\t\t\t*** PASSED FIRST LEG ***"));
        eventMap.put("event2", new PrintCommand("\t\t\t*** HALF WAY THERE (living on a prayer) ***"));
        eventMap.put("event3", new PrintCommand("\t\t\t*** ALMOST, I SWEAR ***"));
        eventMap.put("event4", new PrintCommand("\t\t\t*** ARRIVED AT DESTINATION ***"));

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