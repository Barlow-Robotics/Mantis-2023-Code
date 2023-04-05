// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.ArmPathGenerator;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.DriveToGamePiece;
import frc.robot.commands.FastMoveToHomeFromMiddle;
import frc.robot.commands.FastMoveToHomeFromTop;
import frc.robot.commands.FastMoveToMiddleFromHome;
import frc.robot.commands.FastMoveToTopFromHome;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.commands.MoveArm;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.PathFromCurrentLocation;
import frc.robot.commands.Pivot;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;;

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

    // The current trajectory that will be sent to the filed object for
    // debug/instrumentation
    PathPlannerTrajectory currentTrajectory = null;

    AutoBalance autoBalanceTest;

    public RobotContainer() {
        configureButtonBindings();
        // loadPaths();
        setupInstrumentation();
        buildAutoOptions();

        // autoBalanceTest = new AutoBalance(driveSub);
        // SmartDashboard.putData("auto balance", autoBalanceTest);

        driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, clawSub, visionSub, armSub, autoAlignButton,
                        toggleTargetButton, driverController,
                        throttleJoystickID, turnJoystickID));
    }

    private ArmPathGenerator getFloorCommand() {
        ArmPathGenerator cmd = new ArmPathGenerator(Arm.Position.Floor, armSub);
        cmd.setName("Move to floor");
        return cmd;
    }

    private ArmPathGenerator getHomeCommand() {
        ArmPathGenerator cmd = new ArmPathGenerator(Arm.Position.Home, armSub);
        cmd.setName("Move to home");
        return cmd;
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

        operatorToggleClawButton = new JoystickButton(operatorButtonController,
                XboxControllerConstants.ButtonY);
        operatorToggleClawButton.onTrue(toggleClawCmd);

        /* * * * * * ARM BUTTONS * * * * * */

        moveToRestingPositionButton = new JoystickButton(operatorButtonController,
                XboxControllerConstants.ButtonA);
        moveToRestingPositionButton.onTrue(new ArmPathGenerator(Arm.Position.Home, armSub));

        moveToBottomButton = new JoystickButton(operatorButtonController,
                XboxControllerConstants.LeftStick);
        moveToBottomButton.onTrue(new ArmPathGenerator(Arm.Position.Bottom, armSub));

        moveToMiddleButton = new JoystickButton(operatorButtonController,
                XboxControllerConstants.LeftBumper);
        moveToMiddleButton.onTrue(new ArmPathGenerator(Arm.Position.Middle, armSub));

        moveToTopButton = new JoystickButton(operatorButtonController,
                XboxControllerConstants.ButtonX);
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
        stringChooser.setDefaultOption("Place on Top, Leave Community", "placeTopAndReverse");
        stringChooser.addOption("Place on Top, Engage Station", "placeTopAndEngage");
        stringChooser.addOption("Place on Top, Leave Short Community, Pick Up Game Piece",
                "placeTopAndGrabPieceShortSide");
        stringChooser.addOption("Place on Top, Leave Long Community, Pick Up Game Piece",
                "placeTopAndGrabPieceLongSide");
        SmartDashboard.putData("String Chooser", stringChooser);
    }

    private PathPlannerTrajectory loadPath(String name, double velocity, double accel, boolean reverse) {
        PathPlannerTrajectory temp = PathPlanner.loadPath(
                name,
                new PathConstraints(velocity, accel),
                reverse);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(temp, DriverStation.getAlliance());
    }

    /* * * * * * REVERSE * * * * * */

    InstrumentedSequentialCommandGroup createPlaceTopAndReverseCommand() {
        /* Place Game Piece on Bottom Row, Reverse Out of Community */
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        reversePath = loadPath(
                "Reverse", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, true);

        theCommand.addCommands(new InstantCommand(() -> clawSub.close()));
        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = reversePath));
        theCommand
                .addCommands(new InstantCommand(
                        () -> driveSub.resetOdometry(reversePath.getInitialPose()), driveSub));
        theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        theCommand.addCommands(new ArmPathGenerator(Position.Floor, armSub).getPathFromHome());

        // theCommand.addCommands(
        // new FastMoveToTopFromHome(
        // armSub,
        // ArmConstants.TopArmAngle,
        // ArmConstants.RotateVel,
        // ArmConstants.RotateAccel,
        // ArmConstants.TopArmLength,
        // ArmConstants.ExtendVel,
        // ArmConstants.ExtendAccel
        // )
        // );

        theCommand.addCommands(new WaitCommand(0.25));
        theCommand.addCommands(new OpenClaw(clawSub));
        theCommand.addCommands(new WaitCommand(0.5));
        theCommand.addCommands(new ArmPathGenerator(Position.Home, armSub).getPathFromFloor());
        theCommand.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        theCommand.addCommands(new PPRamseteCommand(
                reversePath,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                driveSub::setSpeeds,
                false,
                driveSub));

        theCommand.onCommandInitialize(Robot::reportCommandStart);
        theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCommand;
    }

    /* * * * * * ENGAGE * * * * * */

    InstrumentedSequentialCommandGroup createPlaceTopAndEngageCommand() {
        /* Place Game Piece on Bottom Row, Reverse Out of Community */
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        engagePath = loadPath(
                "ChargingStation", 1.0, DriveConstants.DefaultAutoAccel, true);

        theCommand.addCommands(new InstantCommand(() -> clawSub.close()));
        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = engagePath));
        theCommand.addCommands(new InstantCommand(() -> driveSub.resetOdometry(engagePath.getInitialPose()),
                driveSub));
        // theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));

        // theCommand.addCommands(new ArmPathGenerator(Position.Floor,
        // armSub).getPathFromHome());
        // theCommand.addCommands(
        // new FastMoveToTopFromHome(
        // armSub,
        // ArmConstants.TopArmAngle,
        // ArmConstants.RotateVel,
        // ArmConstants.RotateAccel,
        // ArmConstants.TopArmLength,
        // ArmConstants.ExtendVel,
        // ArmConstants.ExtendAccel));

        // theCommand.addCommands(new WaitCommand(0.5));
        // theCommand.addCommands(new OpenClaw(clawSub));
        // theCommand.addCommands(new WaitCommand(0.5));

        // theCommand.addCommands(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromFloor());
        // theCommand.addCommands(
        // new FastMoveToHomeFromTop(
        // armSub,
        // ArmConstants.HomeArmAngle,
        // ArmConstants.RotateVel,
        // ArmConstants.RotateAccel,
        // ArmConstants.HomeArmLength,
        // ArmConstants.ExtendVel,
        // ArmConstants.ExtendAccel));

        theCommand.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        theCommand.addCommands(new PPRamseteCommand(
                engagePath,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                driveSub::setSpeeds,
                false,
                driveSub));

        theCommand.addCommands(new AutoBalance(driveSub));

        theCommand.onCommandInitialize(Robot::reportCommandStart);
        theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCommand;
    }

    /* * * * * * GRAB PIECE SHORT COMMUNITY SIDE * * * * * */

    InstrumentedSequentialCommandGroup createPlaceTopAndGrabPieceShortSideCommand() {
        // Place GP on Top, Reverse over Short Community Line, Grab GP, Place GP on Mid
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        shortSideGamePiecePath1 = loadPath(
                "GrabPieceShortSide1", 1.0, DriveConstants.DefaultAutoAccel, true);

        theCommand.addCommands(new InstantCommand(() -> clawSub.close()));
        theCommand.addCommands(
                new InstantCommand(
                        () -> driveSub.resetOdometry(shortSideGamePiecePath1.getInitialPose()),
                        driveSub));
        theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));

        // theCommand.addCommands(new ArmPathGenerator(Position.Top,
        // armSub).getPathFromHome());
        theCommand.addCommands(
                new FastMoveToTopFromHome(
                        armSub,
                        ArmConstants.TopArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.TopArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new WaitCommand(0.5));
        theCommand.addCommands(new OpenClaw(clawSub));

        // theCommand.addCommands(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromTop());
        theCommand.addCommands(
                new FastMoveToHomeFromTop(
                        armSub,
                        ArmConstants.HomeArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.HomeArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new PPRamseteCommand(
                shortSideGamePiecePath1,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                driveSub::setSpeeds,
                false,
                driveSub).alongWith(new ArmPathGenerator(Position.Floor, armSub).getPathFromHome()));

        ParallelCommandGroup pg = new ParallelCommandGroup();
        pg.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        pg.addCommands(new DriveToGamePiece(1.0, 1.0, driveSub, visionSub, clawSub, false));
        theCommand.addCommands(pg);

        if (DriverStation.getAlliance() == Alliance.Red) {
            theCommand.addCommands(new Pivot(driveSub, 90.0, 1.0));
        } else {
            theCommand.addCommands(new Pivot(driveSub, -90.0, 1.0));
        }

        // shortSideGamePiecePath2 = loadPath(
        // "GrabPieceShortSide2", DriveConstants.DefaultAutoVelocity,
        // DriveConstants.DefaultAutoAccel, false
        // );

        // theCommand.addCommands(new PPRamseteCommand(
        // shortSideGamePiecePath2,
        // driveSub::getPose,
        // new RamseteController(),
        // new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
        // driveSub::setSpeeds,
        // false,
        // driveSub).alongWith(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromFloor()));

        // In order to create path from current location, the following points were
        // manually
        // created from the waypoints in the PathPlanner tool. If the path is updated in
        // PathPlanner,
        // then these points will need to be manually updated accordingly. (wpk)
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(new Translation2d(4.37, 4.59), new Rotation2d(Math.PI)));
        points.add(new PathPoint(new Translation2d(2.15, 4.40), new Rotation2d(Math.PI)));

        theCommand.addCommands(
                new PathFromCurrentLocation(
                        points,
                        new PathConstraints(DriveConstants.DefaultAutoVelocity,
                                DriveConstants.DefaultAutoAccel),
                        false,
                        driveSub::getPose,
                        new RamseteController(),
                        new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                        driveSub::setSpeeds,
                        true,
                        driveSub,
                        driveSub)
                        .alongWith(new ArmPathGenerator(Position.Home, armSub)
                                .getPathFromFloor()));

        // theCommand.addCommands(new ArmPathGenerator(Position.Middle,
        // armSub).getPathFromHome());
        theCommand.addCommands(
                new FastMoveToMiddleFromHome(
                        armSub,
                        ArmConstants.MiddleArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.MiddleArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        theCommand.addCommands(new OpenClaw(clawSub));

        // theCommand.addCommands(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromMiddle());
        theCommand.addCommands(
                new FastMoveToHomeFromMiddle(
                        armSub,
                        ArmConstants.HomeArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.HomeArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.onCommandInitialize(Robot::reportCommandStart);
        theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCommand;
    }

    /* * * * * * GRAB PIECE LONG COMMUNITY SIDE * * * * * */

    InstrumentedSequentialCommandGroup createPlaceTopAndGrabPieceLongSideCommand() {
        // Place GP on Top, Reverse over Long Community Line, Grab GP, Place GP on Mid
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        longSideGamePiecePath1 = loadPath(
                "GrabPieceLongSide1", 1.0, DriveConstants.DefaultAutoAccel, true);

        theCommand.addCommands(new InstantCommand(() -> clawSub.close()));
        theCommand.addCommands(
                new InstantCommand(
                        () -> driveSub.resetOdometry(longSideGamePiecePath1.getInitialPose()),
                        driveSub));
        theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));

        // theCommand.addCommands(new ArmPathGenerator(Position.Top,
        // armSub).getPathFromHome());
        theCommand.addCommands(
                new FastMoveToTopFromHome(
                        armSub,
                        ArmConstants.TopArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.TopArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new WaitCommand(0.5));
        theCommand.addCommands(new OpenClaw(clawSub));

        // theCommand.addCommands(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromTop());
        theCommand.addCommands(
                new FastMoveToHomeFromTop(
                        armSub,
                        ArmConstants.HomeArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.HomeArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new PPRamseteCommand(
                longSideGamePiecePath1,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                driveSub::setSpeeds,
                false,
                driveSub).alongWith(new ArmPathGenerator(Position.Floor, armSub).getPathFromHome()));

        theCommand.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        theCommand.addCommands(new DriveToGamePiece(1.0, 1.5, driveSub, visionSub, clawSub, true)); // wpk need
                                                                                                    // to put
                                                                                                    // in
                                                                                                    // correct
                                                                                                    // distance

        // longSideGamePiecePath2 = loadPath(
        // "GrabPieceLongSide2", DriveConstants.DefaultAutoVelocity,
        // DriveConstants.DefaultAutoAccel, false);

        // theCommand.addCommands(new PPRamseteCommand(
        // longSideGamePiecePath2,
        // driveSub::getPose,
        // new RamseteController(),
        // new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
        // driveSub::setSpeeds,
        // false,
        // driveSub).alongWith(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromFloor()));

        var turnAngle = 110.0;
        if (DriverStation.getAlliance() == Alliance.Red) {
            theCommand.addCommands(new Pivot(driveSub, turnAngle, 1.0));
        } else {
            theCommand.addCommands(new Pivot(driveSub, -turnAngle, 1.0));
        }

        // In order to create path from current location, the following points were
        // manually
        // created from the waypoints in the PathPlanner tool. If the path is updated in
        // PathPlanner,
        // then these points will need to be manually updated accordingly. (wpk)
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        points.add(new PathPoint(new Translation2d(4.11, 1.0), new Rotation2d(Math.toRadians(180.0))));
        points.add(new PathPoint(new Translation2d(1.97, 1.03), new Rotation2d(Math.PI)));

        theCommand.addCommands(
                new PathFromCurrentLocation(
                        points,
                        new PathConstraints(DriveConstants.DefaultAutoVelocity,
                                DriveConstants.DefaultAutoAccel),
                        false,
                        driveSub::getPose,
                        new RamseteController(),
                        new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
                        driveSub::setSpeeds,
                        true,
                        driveSub,
                        driveSub)
                        .alongWith(new ArmPathGenerator(Position.Floor, armSub)
                                .getPathFromHome()));

        // theCommand.addCommands(new ArmPathGenerator(Position.Middle,
        // armSub).getPathFromHome());
        theCommand.addCommands(
                new FastMoveToMiddleFromHome(
                        armSub,
                        ArmConstants.MiddleArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.MiddleArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        theCommand.addCommands(new OpenClaw(clawSub));

        // theCommand.addCommands(new ArmPathGenerator(Position.Home,
        // armSub).getPathFromMiddle());
        theCommand.addCommands(
                new FastMoveToHomeFromMiddle(
                        armSub,
                        ArmConstants.HomeArmAngle,
                        ArmConstants.RotateVel,
                        ArmConstants.RotateAccel,
                        ArmConstants.HomeArmLength,
                        ArmConstants.ExtendVel,
                        ArmConstants.ExtendAccel));

        theCommand.onCommandInitialize(Robot::reportCommandStart);
        theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCommand;
    }

    private void setupInstrumentation() {

        // ArrayList<Translation2d> gamePiecePoints = new ArrayList<Translation2d>();
        // gamePiecePoints.add(new Translation2d(7.09, 4.61));
        // gamePiecePoints.add(new Translation2d(7.09, 0.9));
        // gamePiecePoints.add(new Translation2d(9.51, 4.57));
        // gamePiecePoints.add(new Translation2d(9.51, 0.9));

        // placeTopAndReverse = createPlaceTopAndReverseCommand();
        // placeTopAndEngage = createPlaceTopAndEngageCommand();
        // placeTopAndGrabPieceShortSide = createPlaceTopAndGrabPieceShortSideCommand();
        // placeTopAndGrabPieceLongSide = createPlaceTopAndGrabPieceLongSideCommand();

        PPRamseteCommand.setLoggingCallbacks(
                (PathPlannerTrajectory traj) -> {
                    this.currentTrajectory = traj;
                },
                (Pose2d targetPose) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
                            .setDouble(targetPose.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
                            .setDouble(targetPose.getY());
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

        PathFromCurrentLocation.setLoggingCallbacks(
                (PathPlannerTrajectory traj) -> {
                    this.currentTrajectory = traj;
                },
                (Pose2d targetPose) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
                            .setDouble(targetPose.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
                            .setDouble(targetPose.getY());
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
    }

    public Command getAutonomousCommand() {
        // return createPlaceTopAndEngageCommand() ;
        // return this.createPlaceTopAndGrabPieceLongSideCommand() ;
        // return this.createPlaceTopAndGrabPieceShortSideCommand() ;
        // return this.createPlaceTopAndReverseCommand() ;
        // return this.placeTopAndGrabPieceShortSide;
        // return autoChooser.getSelected();

        // return new AutoBalance(driveSub) ;

        this.currentTrajectory = new PathPlannerTrajectory();

        String choice = stringChooser.getSelected();
        if (choice == "placeTopAndReverse") {
            return createPlaceTopAndReverseCommand();
        } else if (choice == "placeTopAndEngage") {
            return createPlaceTopAndEngageCommand();
        } else if (choice == "placeTopAndGrabPieceShortSide") {
            return createPlaceTopAndGrabPieceShortSideCommand();
        } else if (choice == "placeTopAndGrabPieceLongSide") {
            return createPlaceTopAndGrabPieceLongSideCommand();
        } else {
            return null;
        }
    }

    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }
}