// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    SequentialCommandGroup placeTopAndEngage;
    SequentialCommandGroup placeTopAndReverse;
    SequentialCommandGroup placeTopAndGrabPieceShortSide;
    SequentialCommandGroup placeTopAndGrabPieceLongSide;

    AutoBalance autoBalanceTest;

    public RobotContainer() {
        configureButtonBindings();
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

    private void createAutonomousCommands() {
        ArmPathGenerator toTop = new ArmPathGenerator(Position.Top, armSub);
        ArmPathGenerator toResting = new ArmPathGenerator(Position.Resting, armSub);
        ArmPathGenerator toFloor = new ArmPathGenerator(Position.Floor, armSub);
        ArmPathGenerator toMiddle = new ArmPathGenerator(Position.Middle, armSub);

        ArrayList<Translation2d> gamePiecePoints = new ArrayList<Translation2d>();
        gamePiecePoints.add(new Translation2d(7.09, 4.61));
        gamePiecePoints.add(new Translation2d(7.09, 0.9));
        gamePiecePoints.add(new Translation2d(9.51, 4.57));
        gamePiecePoints.add(new Translation2d(9.51, 0.9));

        PathPlannerTrajectory reversePath = PathPlanner.loadPath(
                "Reverse",
                new PathConstraints(1, 4),
                true);

        PathPlannerTrajectory engagePath = PathPlanner.loadPath(
                "ChargingStation",
                new PathConstraints(1, 4),
                true);

        PathPlannerTrajectory shortSideGamePiecePath1 = PathPlanner.loadPath(
                "GrabPieceShortSide1",
                new PathConstraints(1, 4),
                true);

        PathPlannerTrajectory shortSideGamePiecePath2 = PathPlanner.loadPath(
                "GrabPieceShortSide2",
                new PathConstraints(1, 4),
                false);

        PathPlannerTrajectory longSideGamePiecePath1 = PathPlanner.loadPath(
                "GrabPieceLongSide1",
                new PathConstraints(1, 4),
                true);

        PathPlannerTrajectory longSideGamePiecePath2 = PathPlanner.loadPath(
                "GrabPieceLongSide2",
                new PathConstraints(1, 4),
                false);

        RamseteController controller = new RamseteController();

        /*
         * Place Game Piece on Top Row, Reverse Out of Community, Engage Charging
         * Station
         */
        placeTopAndEngage = new SequentialCommandGroup();
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndEngage.addCommands(toTop.getPathFromResting());
        placeTopAndEngage.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndEngage.addCommands(new OpenClaw(clawSub));
        placeTopAndEngage.addCommands(toResting.getPathFromTop());
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndEngage.addCommands(new InstantCommand(() -> driveSub.resetOdometry(engagePath.getInitialPose()), driveSub));
        placeTopAndEngage.addCommands(new PPRamseteCommand(
                engagePath,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub));
        placeTopAndEngage.addCommands(new AutoBalance(driveSub));

        /* Place Game Piece on Top Row, Reverse Out of Community */
        placeTopAndReverse = new SequentialCommandGroup();
        // placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        // placeTopAndReverse.addCommands(toTop.getPathFromResting());
        // placeTopAndReverse.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        // placeTopAndReverse.addCommands(new OpenClaw(clawSub));
        // placeTopAndReverse.addCommands(toResting.getPathFromTop());
        placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndReverse.addCommands(new InstantCommand(() -> driveSub.resetOdometry(reversePath.getInitialPose()), driveSub));
        placeTopAndReverse.addCommands(new PPRamseteCommand(
                reversePath,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub));

        /*
         * Place Game Piece on Top Row, Reverse over Short Community Line, Grab Game
         * Piece, Place Game Piece on Middle Row
         */
        placeTopAndGrabPieceShortSide = new SequentialCommandGroup();
        // placeTopAndGrabPieceShortSide.addCommands(new InstantCommand(() ->
        // clawSub.disableAutoClose()));
        // placeTopAndGrabPieceShortSide.addCommands(toTop.getPathFromResting());
        // placeTopAndGrabPieceShortSide.addCommands(new
        // edu.wpi.first.wpilibj2.command.WaitCommand(2));
        // placeTopAndGrabPieceShortSide.addCommands(new OpenClaw(clawSub));
        // placeTopAndGrabPieceShortSide.addCommands(toResting.getPathFromTop());
        placeTopAndGrabPieceShortSide.addCommands(
                new InstantCommand(() -> driveSub.resetOdometry(shortSideGamePiecePath1.getInitialPose()), driveSub));
        placeTopAndGrabPieceShortSide.addCommands(new PPRamseteCommand(
                shortSideGamePiecePath1,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub)/* .alongWith(toFloor.getPathFromResting()) */);
        // placeTopAndGrabPieceShortSide.addCommands(new DriveToGamePiece(driveSub,
        // visionSub));
        // placeTopAndGrabPieceShortSide.addCommands(new InstantCommand(() ->
        // clawSub.enableAutoClose()));
        // placeTopAndGrabPieceShortSide.addCommands(new PPRamseteCommand(
        // shortSideGamePiecePath2,
        // driveSub::getPose,
        // controller,
        // new DifferentialDriveKinematics(0.75),
        // driveSub::setSpeeds,
        // true,
        // driveSub
        // ).alongWith(toMiddle.getPathFromFloor()));
        // placeTopAndGrabPieceShortSide.addCommands(toResting.getPathFromMiddle());

        /*
         * Place Game Piece on Top Row, Reverse over Long Community Line, Grab Game
         * Piece, Place Game Piece on Middle Row
         */
        placeTopAndGrabPieceLongSide = new SequentialCommandGroup();
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndGrabPieceLongSide.addCommands(toTop.getPathFromResting());
        placeTopAndGrabPieceLongSide.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndGrabPieceLongSide.addCommands(new OpenClaw(clawSub));
        placeTopAndGrabPieceLongSide.addCommands(toResting.getPathFromTop());
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> driveSub.resetOdometry(null), driveSub));
        placeTopAndGrabPieceLongSide.addCommands(new PPRamseteCommand(
                longSideGamePiecePath1,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub).alongWith(toFloor.getPathFromResting()));
        placeTopAndGrabPieceLongSide.addCommands(new DriveToGamePiece(driveSub, visionSub));
        placeTopAndGrabPieceLongSide.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndGrabPieceLongSide.addCommands(new PPRamseteCommand(
                longSideGamePiecePath2,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub).alongWith(toMiddle.getPathFromFloor()));
        placeTopAndGrabPieceLongSide.addCommands(toResting.getPathFromMiddle());

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