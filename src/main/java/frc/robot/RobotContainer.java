// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.ArmPathGenerator;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.DriveToGamePiece;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* Subsystems */
    public final Drive driveSub = new Drive();
    public final Arm armSub = new Arm();
    public final Claw clawSub = new Claw(armSub);
    public final Vision visionSub = new Vision();

    /* Commands */
    private final ToggleClaw toggleClawCmd = new ToggleClaw(clawSub);

    /* Controllers */
    public Joystick driverController; // Joystick 1
    public Joystick operatorButtonController; // Joystick 2
    public Joystick operatorAxisController; // Joystick 3

    /* Buttons */
    private Trigger moveToTopButton; // left stick (blue button)
    private Trigger moveToMiddleButton; // left bumper (green button)
    private Trigger moveToBottomButton; // button a (left white button)
    private Trigger moveToRestingPositionButton; // button x (middle white button)
    private Trigger moveToPlayerStationButton; // right stick (black button)
    private Trigger moveToFloorButton; // right bumper (yellow button)
    private Trigger driverToggleClawButton;
    private Trigger operatorToggleClawButton; // y button (right white button)
    public Trigger toggleTargetButton;
    public Trigger autoAlignButton;

    /* Drive & Arm Movement */
    public int throttleJoystickID;
    public int turnJoystickID;
    public int angleJoystickID;
    public int extensionJoystickID;

    /* Shuffleboard */
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    final SendableChooser<String> stringChooser = new SendableChooser<String>();

    PathPlannerTrajectory reversePath;
    PathPlannerTrajectory engagePath;
    PathPlannerTrajectory shortSideGamePiecePath1;
    PathPlannerTrajectory shortSideGamePiecePath2;
    PathPlannerTrajectory longSideGamePiecePath1;
    PathPlannerTrajectory longSideGamePiecePath2;

    SequentialCommandGroup placeTopAndEngage;
    SequentialCommandGroup placeTopAndReverse;
    SequentialCommandGroup placeTopAndGrabPieceShortSide;
    SequentialCommandGroup placeTopAndGrabPieceLongSide;

    AutoBalance autoBalanceTest;

    public RobotContainer() {
        configureButtonBindings();
        loadPaths();
        createAutonomousCommands();
        buildAutoOptions();

        autoBalanceTest = new AutoBalance(driveSub);
        SmartDashboard.putData("auto balance", autoBalanceTest);

        driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, clawSub, visionSub, autoAlignButton, toggleTargetButton, driverController,
                        throttleJoystickID, turnJoystickID));

        // armSub.setDefaultCommand(
        // new MoveArmManual(armSub));
    }

    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorButtonController = new Joystick(2);
        operatorAxisController = new Joystick(3);

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);
        System.out.println("The controller name is " + DriverStation.getJoystickName(1));

        // xAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        // yawAxis = Constants.LogitechDualActionConstants.RightJoystickX;
        throttleJoystickID = Constants.RadioMasterConstants.LeftGimbalY;
        turnJoystickID = Constants.RadioMasterConstants.RightGimbalX;
        angleJoystickID = Constants.LogitechDualActionConstants.LeftJoystickY;
        extensionJoystickID = Constants.LogitechDualActionConstants.RightJoystickX;

        /* * * * * * CLAW BUTTONS * * * * * */

        driverToggleClawButton = new JoystickButton(driverController, RadioMasterConstants.ButtonA);
        driverToggleClawButton.onTrue(toggleClawCmd);

        operatorToggleClawButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonY);
        operatorToggleClawButton.onTrue(toggleClawCmd);

        /* * * * * * ARM BUTTONS * * * * * */

        moveToRestingPositionButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonA);
        moveToRestingPositionButton.onTrue(new ArmPathGenerator(Arm.Position.Resting, armSub));

        moveToBottomButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftStick);
        moveToBottomButton.onTrue(new ArmPathGenerator(Arm.Position.Bottom, armSub));

        moveToMiddleButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftBumper);
        moveToMiddleButton.onTrue(new ArmPathGenerator(Arm.Position.Middle, armSub));

        moveToTopButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonX);
        moveToTopButton.onTrue(new ArmPathGenerator(Arm.Position.Top, armSub));

        moveToFloorButton = new JoystickButton(operatorButtonController, XboxControllerConstants.RightBumper);
        moveToFloorButton.onTrue(new ArmPathGenerator(Arm.Position.Floor, armSub));

        moveToPlayerStationButton = new JoystickButton(operatorButtonController, XboxControllerConstants.RightStick);
        moveToPlayerStationButton.onTrue(new ArmPathGenerator(Arm.Position.PlayerStation, armSub));

        /* * * * * * VISION BUTTONS * * * * * */

        toggleTargetButton = new JoystickAnalogButton(driverController, 6, 0.0, 1.0);
        autoAlignButton = new JoystickButton(driverController, RadioMasterConstants.ButtonD);
    }

    private void buildAutoOptions() {
        stringChooser.setDefaultOption("Place on Top and Leave Community", "test 1");
        stringChooser.addOption("Place on Top and Engage Station", "test 2");
        SmartDashboard.putData("String Chooser", stringChooser);
    }

    private void loadPaths() {
        reversePath = PathPlanner.loadPath(
                "Reverse",
                new PathConstraints(1, 4),
                true);

        engagePath = PathPlanner.loadPath(
                "ChargingStation",
                new PathConstraints(1, 4),
                true);

        shortSideGamePiecePath1 = PathPlanner.loadPath(
                "GrabPieceShortSide1",
                new PathConstraints(1, 4),
                true);

        shortSideGamePiecePath2 = PathPlanner.loadPath(
                "GrabPieceShortSide2",
                new PathConstraints(1, 4),
                false);

        longSideGamePiecePath1 = PathPlanner.loadPath(
                "GrabPieceLongSide1",
                new PathConstraints(1, 4),
                true);

        longSideGamePiecePath2 = PathPlanner.loadPath(
                "GrabPieceLongSide2",
                new PathConstraints(1, 4),
                false);

    }

    SequentialCommandGroup createPlaceTopAndReverseCommand() {
        /* Place Game Piece on Top Row, Reverse Out of Community */
        SequentialCommandGroup placeTopAndReverse = new SequentialCommandGroup();
        placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndReverse.addCommands(new ArmPathGenerator(Position.Top,
                armSub).getPathFromResting());
        placeTopAndReverse.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndReverse.addCommands(new OpenClaw(clawSub));
        placeTopAndReverse.addCommands(new ArmPathGenerator(Position.Resting,
                armSub).getPathFromTop());
        placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));

        PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(reversePath.getState(0),
                DriverStation.getAlliance());
        placeTopAndReverse
                .addCommands(new InstantCommand(() -> driveSub.resetOdometry(initialState.poseMeters), driveSub));

        PPRamseteCommand reverseCommand = new PPRamseteCommand(
                reversePath,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                // false,
                driveSub);

        placeTopAndReverse.addCommands(reverseCommand);

        return placeTopAndReverse;
    }

    SequentialCommandGroup createPlaceTopAndEngageCommand() {
        /* Place Game Piece on Top Row, Reverse Out of Community */
        SequentialCommandGroup placeTopAndEngage = new SequentialCommandGroup();
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndEngage.addCommands(new ArmPathGenerator(Position.Top,
                armSub).getPathFromResting());
        placeTopAndEngage.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndEngage.addCommands(new OpenClaw(clawSub));
        placeTopAndEngage.addCommands(new ArmPathGenerator(Position.Resting,
                armSub).getPathFromTop());
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));

        placeTopAndEngage
                .addCommands(new InstantCommand(() -> driveSub.resetOdometry(engagePath.getInitialPose()), driveSub));
        placeTopAndEngage.addCommands(new PPRamseteCommand(
                engagePath,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub));
        placeTopAndEngage.addCommands(new AutoBalance(driveSub));

        return placeTopAndEngage;
    }

    SequentialCommandGroup createPlaceTopAndGrabPieceShortSideCommand() {
        // Place GP on Top, Reverse over Short Community Line, Grab GP, Place GP on Mid
        placeTopAndGrabPieceShortSide = new SequentialCommandGroup();
        //
        // placeTopAndGrabPieceShortSide.addCommands(new InstantCommand(() ->
        // clawSub.disableAutoClose()));
        // placeTopAndGrabPieceShortSide.addCommands(new ArmPathGenerator(Position.Top,
        // armSub).getPathFromResting());
        // placeTopAndGrabPieceShortSide.addCommands(new
        // edu.wpi.first.wpilibj2.command.WaitCommand(2));
        // placeTopAndGrabPieceShortSide.addCommands(new OpenClaw(clawSub));
        // placeTopAndGrabPieceShortSide.addCommands(new
        // ArmPathGenerator(Position.Resting,
        // armSub).getPathFromTop());
        //
        placeTopAndGrabPieceShortSide.addCommands(
                new InstantCommand(() -> driveSub.resetOdometry(shortSideGamePiecePath1.getInitialPose()), driveSub));
        placeTopAndGrabPieceShortSide.addCommands(new PPRamseteCommand(
                shortSideGamePiecePath1,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub)/* .alongWith(toFloor.getPathFromResting()) */);
        //
        // placeTopAndGrabPieceShortSide.addCommands(new DriveToGamePiece(driveSub,
        // visionSub));
        // placeTopAndGrabPieceShortSide.addCommands(new InstantCommand(() ->
        // clawSub.enableAutoClose()));
        // placeTopAndGrabPieceShortSide.addCommands(new PPRamseteCommand(
        // shortSideGamePiecePath2,
        // driveSub::getPose,
        // new RamseteController(),
        // new DifferentialDriveKinematics(0.75),
        // driveSub::setSpeeds,
        // true,
        // driveSub).alongWith(new ArmPathGenerator(Position.Middle,
        // armSub).getPathFromFloor()));
        // placeTopAndGrabPieceShortSide.addCommands(new
        // ArmPathGenerator(Position.Resting, armSub).getPathFromMiddle());
        //
        return placeTopAndGrabPieceShortSide;
    }

    SequentialCommandGroup createPlaceTopAndGrabPieceLongSideCommand() {
        // Place GP on Top, Reverse over Long Community Line, Grab GP, Place GP on Mid

        placeTopAndGrabPieceLongSide = new SequentialCommandGroup();
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndGrabPieceLongSide.addCommands(new ArmPathGenerator(Position.Top, armSub).getPathFromResting());
        placeTopAndGrabPieceLongSide.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndGrabPieceLongSide.addCommands(new OpenClaw(clawSub));
        placeTopAndGrabPieceLongSide.addCommands(new ArmPathGenerator(Position.Resting, armSub).getPathFromTop());
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> driveSub.resetOdometry(null), driveSub));
        placeTopAndGrabPieceLongSide.addCommands(new PPRamseteCommand(
                longSideGamePiecePath1,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub).alongWith(new ArmPathGenerator(Position.Floor, armSub).getPathFromResting()));
        placeTopAndGrabPieceLongSide.addCommands(new DriveToGamePiece(driveSub, visionSub));
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndGrabPieceLongSide.addCommands(new PPRamseteCommand(
                longSideGamePiecePath2,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub).alongWith(new ArmPathGenerator(Position.Middle, armSub).getPathFromFloor()));
        placeTopAndGrabPieceLongSide.addCommands(new ArmPathGenerator(Position.Resting, armSub).getPathFromMiddle());

        return placeTopAndGrabPieceLongSide;
    }

    private void createAutonomousCommands() {

        ArrayList<Translation2d> gamePiecePoints = new ArrayList<Translation2d>();
        gamePiecePoints.add(new Translation2d(7.09, 4.61));
        gamePiecePoints.add(new Translation2d(7.09, 0.9));
        gamePiecePoints.add(new Translation2d(9.51, 4.57));
        gamePiecePoints.add(new Translation2d(9.51, 0.9));

        placeTopAndReverse = createPlaceTopAndReverseCommand();
        placeTopAndEngage = createPlaceTopAndEngageCommand();
        placeTopAndGrabPieceShortSide = createPlaceTopAndGrabPieceShortSideCommand();
        placeTopAndGrabPieceLongSide = createPlaceTopAndGrabPieceLongSideCommand();

        PPRamseteCommand.setLoggingCallbacks(
                (PathPlannerTrajectory activeTrajectory) -> {
                    System.out.println("PP Logging: activeTRajectory is " + activeTrajectory);
                },
                (Pose2d targetPose) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X").setDouble(targetPose.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y").setDouble(targetPose.getY());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
                            .setDouble(targetPose.getRotation().getDegrees());
                },
                (ChassisSpeeds setpointSpeeds) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
                            .setDouble(setpointSpeeds.vxMetersPerSecond);
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
                            .setDouble(setpointSpeeds.vyMetersPerSecond);
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
                            .setDouble(setpointSpeeds.omegaRadiansPerSecond);
                },
                (Translation2d translationError, Rotation2d rotationError) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
                            .setDouble(translationError.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
                            .setDouble(translationError.getY());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
                            .setDouble(rotationError.getDegrees());
                });

        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees").setDouble(0.0);

        autoChooser.setDefaultOption("Place on Top, Leave Community", placeTopAndReverse);
        autoChooser.addOption("Place on Top, Engage Station", placeTopAndEngage);
        autoChooser.addOption("Place on Top, Leave Short Community, Pick Up Game Piece", placeTopAndGrabPieceShortSide);
        autoChooser.addOption("Place on Top, Leave Long Community, Pick Up Game Piece", placeTopAndGrabPieceLongSide);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return this.placeTopAndReverse;
        // return autoChooser.getSelected();
    }
}