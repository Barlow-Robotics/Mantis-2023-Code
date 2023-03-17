// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

// import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
// import frc.robot.commands.AlignWithAprilTags;
// import frc.robot.commands.AlignWithGamePiece;
// import frc.robot.commands.AlignWithPole;
import frc.robot.commands.ArmPathGenerator;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Vision.AlignType;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    public final Drive driveSub = new Drive();
    public final Arm armSub = new Arm();
    public final Claw clawSub = new Claw(armSub);
    public final Vision visionSub = new Vision();
    // private final Underglow underglowSub = new Underglow();

    // private final TurnOffUnderglow turnOffUnderGlowCom = new
    // TurnOffUnderglow(underglowSub);
    // private final TurnOnUnderglow turnOnUnderGlowCom = new
    // TurnOnUnderglow(underglowSub);

    public Joystick driverController; // Joystick 1
    public Joystick operatorButtonController; // Joystick 2
    public Joystick operatorAxisController; // Joystick 3

    PIDController pid;

    private int xAxis;
    private int yawAxis;
    private int angleAxis;
    private int extensionAxis;

    private Trigger moveToTopButton; // left stick (blue button)
    private Trigger moveToMiddleButton; // left bumper (green button)
    private Trigger moveToBottomButton; // button a (left white button)
    private Trigger moveToRestingPositionButton; // button x (middle white button)
    private Trigger moveToPlayerStationButton; // right stick (black button)
    // private Trigger alignWithAprilTagsButton;
    // private Trigger alignWithGamePieceButton;
    // private Trigger alignWithPoleButton;
    private Trigger moveToFloorButton; // right bumper (yellow button)
    private Trigger driverToggleClawButton;
    private Trigger operatorToggleClawButton; // y button (right white button)
    public Trigger changeTargetGamePieceButton;
    public Trigger changeTargetAprilTagButton;
    public Trigger changeTargetPoleButton;
    private Trigger autoAlignButton;

    private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    final SendableChooser<String> stringChooser = new SendableChooser<String>();

    SequentialCommandGroup placeTopAndEngage;
    SequentialCommandGroup placeTopAndReverse;

    private final ToggleClaw toggleClaw = new ToggleClaw(clawSub);

    public RobotContainer() {
        configureButtonBindings();
        createAutonomousCommands();
        buildAutoOptions();

        driveSub.setDefaultCommand( // DriveDefaultCommand
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> {

                            SmartDashboard.putString("Align Type", visionSub.getSelectedAlign().toString()) ;

                            boolean autoSteer = autoAlignButton.getAsBoolean();

                            double x = driverController.getRawAxis(xAxis);
                            if (Math.abs(x) < 0.01) {
                                x = 0.0;
                            }
                            double yaw = -driverController.getRawAxis(yawAxis);
                            if (Math.abs(yaw) < 0.01) {
                                yaw = 0.0;
                            }
                            double speed = -x;

                            // // If we're going forward, use "full" speed
                            // if (speed > 0.0) {
                            // speed = speed * 0.5;
                            // } else {
                            // // we're going backward, so use slower speed
                            // speed = speed * 0.75;
                            // }
                            double turn = -yaw;

                            if (!autoSteer || !clawSub.isOpen()) {
                                yaw = -turn;

                                // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);
                                yawMultiplier = 0.5f;

                                double yawSign = 1.0;
                                if (yaw < 0.0) {
                                    yawSign = -1.0;
                                }
                                yaw = yawSign * (yaw * yaw)
                                        * yawMultiplier;
                                if (Math.abs(yaw) < 0.02f) {
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
                            NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
                            NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

                            driveSub.drive(-speed, yaw * 0.8, true);
                        },
                        driveSub));

        armSub.setDefaultCommand(
            new RunCommand(
            () -> {
                
            }
            armSub));
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
        xAxis = Constants.RadioMasterConstants.LeftGimbalY;
        yawAxis = Constants.RadioMasterConstants.RightGimbalX;
        angleAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        extensionAxis = Constants.LogitechDualActionConstants.RightJoystickX;

        /* * * * * * CLAW BUTTONS * * * * * */

        driverToggleClawButton = new JoystickButton(driverController, RadioMasterConstants.ButtonA);
        driverToggleClawButton.onTrue(toggleClaw);

        operatorToggleClawButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonY);
        operatorToggleClawButton.onTrue(toggleClaw);

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

        changeTargetGamePieceButton = new JoystickAnalogButton(driverController, 2, 0.0, 1.0);
        changeTargetAprilTagButton = new JoystickAnalogButton(driverController, 2, 0.0, 1.0);
        changeTargetPoleButton = new JoystickAnalogButton(driverController, 2, -1.0, 0.0);

        autoAlignButton = new JoystickButton(driverController, RadioMasterConstants.ButtonD);
        autoAlignButton.onTrue(new AutoAlign(visionSub, driveSub));

        // alignWithAprilTagsButton = new JoystickButton(driverController, 6);
        // alignWithAprilTagsButton.onTrue(new AlignWithAprilTags(visionSub, driveSub));

        // alignWithGamePieceButton = new JoystickButton(driverController, RadioMasterConstants.ButtonD);
        // alignWithGamePieceButton.onTrue(new AlignWithGamePiece(visionSub, driveSub));

        // alignWithPoleButton = new JoystickButton(driverController, 8);
        // alignWithPoleButton.onTrue(new AlignWithPole(visionSub, driveSub));
    }

    private void buildAutoOptions() {
        stringChooser.setDefaultOption("Place on Top and Leave Community", "test 1");
        stringChooser.addOption("Place on Top and Engage Station", "test 2");
        SmartDashboard.putData("String Chooser", stringChooser);
    }

    private void createAutonomousCommands() {
        ArmPathGenerator toTop = new ArmPathGenerator(Position.Top, armSub);
        ArmPathGenerator toResting = new ArmPathGenerator(Position.Resting, armSub);

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("MoveArm", toBottomApg.getPathFromResting());
        // eventMap.put("OpenClaw", openClaw);
        // eventMap.put("MoveToResting", toRestingApg.getPathFromBottom());
        // // eventMap.put("AutoBalance", autoBalance);

        PathPlannerTrajectory traj = PathPlanner.loadPath(
                "Reverse",
                new PathConstraints(1, 4),
                true);

        RamseteController controller = new RamseteController();

        // Command resetOdometry = new InstantCommand(() -> {
        // // driveSub.resetEncoders();
        // driveSub.resetOdometry(traj.getInitialPose());
        // });

        // Command pathFollowingCommand = new PPRamseteCommand(
        // traj,
        // driveSub::getPose,
        // controller,
        // new DifferentialDriveKinematics(0.75),
        // driveSub::setSpeeds,
        // true,
        // driveSub);

        // Command followPathWithEvents = new FollowPathWithEvents(
        // pathFollowingCommand,
        // traj.getMarkers(),
        // eventMap);

        placeTopAndEngage = new SequentialCommandGroup();
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndEngage.addCommands(toTop.getPathFromResting());
        placeTopAndEngage.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndEngage.addCommands(new OpenClaw(clawSub));
        placeTopAndEngage.addCommands(toResting.getPathFromTop());
        placeTopAndEngage.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndEngage.addCommands(new InstantCommand(() -> driveSub.resetOdometry(null), driveSub));
        placeTopAndEngage.addCommands(new PPRamseteCommand(
                traj,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub));
        placeTopAndEngage.addCommands(new AutoBalance(driveSub));

        placeTopAndReverse = new SequentialCommandGroup();
        placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.disableAutoClose()));
        placeTopAndReverse.addCommands(toTop.getPathFromResting());
        placeTopAndReverse.addCommands(new edu.wpi.first.wpilibj2.command.WaitCommand(2));
        placeTopAndReverse.addCommands(new OpenClaw(clawSub));
        placeTopAndReverse.addCommands(toResting.getPathFromTop());
        placeTopAndReverse.addCommands(new InstantCommand(() -> clawSub.enableAutoClose()));
        placeTopAndReverse.addCommands(new InstantCommand(() -> driveSub.resetOdometry(null), driveSub));
        placeTopAndReverse.addCommands(new PPRamseteCommand(
                traj,
                driveSub::getPose,
                controller,
                new DifferentialDriveKinematics(0.75),
                driveSub::setSpeeds,
                true,
                driveSub));

        autoChooser.setDefaultOption("Place on Top and Leave Community", placeTopAndEngage);
        autoChooser.addOption("Place on Top and Engage Station", placeTopAndReverse);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}