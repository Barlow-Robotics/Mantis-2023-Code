// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

// import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.commands.ArmPathGenerator;
import frc.robot.commands.MoveArm;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private final Drive driveSub = new Drive();
    public final Arm armSub = new Arm();
    public final Claw clawSub = new Claw(armSub);
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

    private Trigger moveToTopButton; // left stick (blue button)
    private Trigger moveToMiddleButton; // left bumper (green button)
    private Trigger moveToBottomButton; // button a (left white button)
    private Trigger moveToRestingPositionButton; // button x (middle white button)
    private Trigger moveToPlayerStationButton; // right stick (black button)
    private Trigger alignWithAprilTagsButton;
    private Trigger alignWithGamePieceButton;
    private Trigger alignWithPoleButton;
    private Trigger moveToFloorButton; // right bumper (yellow button)
    private Trigger driverToggleClawButton; 
    private Trigger operatorToggleClawButton; // y button (right white button)

    // private Command moveToBottom;
    // private Command moveToMiddle;
    // private Command moveToResting;
    // private Command moveToFloor;

    private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;

    private final ToggleClaw toggleClaw = new ToggleClaw(clawSub);


    private String autoPath = "" ;

    public RobotContainer() {
        configureButtonBindings();

        driveSub.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> {

                            boolean autoSteer = alignWithGamePieceButton.getAsBoolean();

                            double x = driverController.getRawAxis(xAxis);
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
                            speed = -x ;
                            // if (x != 0) {
                            //     speed = (Math.abs(x) / x) * (Math
                            //             .exp(-400.0 * Math.pow(x / 3.0, 4.0)))
                            //             + (-Math.abs(x) / x);
                            // }
                            // speed = speed * 0.5 ;
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

                                // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);

                                yawMultiplier =  0.5f;

                                double yawSign = 1.0 ;
                                if ( yaw < 0.0 ) {
                                    yawSign = -1.0 ;
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
                                yaw = -pid.calculate(visionSub
                                        .gamePieceDistanceFromCenter());
                                lastAutoSteer = true;
                            }
                            NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed) ;
                            NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw) ;

                            driveSub.drive(-speed, yaw * 0.4, false);

                        },
                        driveSub));

        // armSub.setDefaultCommand(
        //         // A split-stick arcade command, with forward/backward controlled by the left
        //         // hand, and turning controlled by the right.
        //         new RunCommand(
        //                 () -> {
        //                     /* Angle */
        //                     double currentAngle = armSub.getAngle();
        //                     double desiredAngle = currentAngle
        //                             + (operatorButtonController.getRawAxis(angleAxis) * ArmConstants.AngleMultiplier);

        //                     if (desiredAngle > ArmConstants.ArmMaxAngle) {
        //                         desiredAngle = ArmConstants.ArmMaxAngle;
        //                     } else if (desiredAngle < ArmConstants.ArmMinAngle) {
        //                         desiredAngle = ArmConstants.ArmMinAngle;
        //                     }

        //                     armSub.setAngle(desiredAngle, ArmConstants.AngleVel, Constants.ArmConstants.AngleAccelerationTime);

        //                     /* Extension */
        //                     double currentLength = armSub.getLength();
        //                     double desiredLength = currentLength + operatorButtonController.getRawAxis(extensionAxis)
        //                             * ArmConstants.LengthMultiplier;

        //                     if (desiredLength > ArmConstants.ArmMaxLength) {
        //                         desiredLength = ArmConstants.ArmMaxLength;
        //                     } else if (desiredLength < ArmConstants.ArmMinLength) {
        //                         desiredLength = ArmConstants.ArmMinLength;
        //                     }

        //                     armSub.setLength(desiredLength, ArmConstants.LengthVel, ArmConstants.LengthAccelTime);

        //                     // if (desiredLength * Math.cos(desiredAngle) <= 0) {
        //                     //     desiredLength = currentLength;
        //                     // }
        //                     // double desiredAngle = currentAngle + operatorButtonController.getRawAxis(1) * Constants.ArmConstants.AngleMultiplier; 
                            
        //                     // if (desiredAngle > Constants.ArmConstants.ArmMaxAngle) {
        //                     //     desiredAngle = Constants.ArmConstants.ArmMaxAngle;
        //                     // } else if (desiredAngle < Constants.ArmConstants.ArmMinAngle) {
        //                     //     desiredAngle = Constants.ArmConstants.ArmMinAngle;
        //                     // }

        //                     // /* Extension */
        //                     // double currentLength = armSub.getLength();
        //                     // double desiredLength = currentLength + operatorButtonController.getRawAxis(2) * Constants.ArmConstants.LengthMultiplier;
                        
        //                     // if (desiredLength > Constants.ArmConstants.ArmMaxLength) {
        //                     //     desiredLength = Constants.ArmConstants.ArmMaxLength;
        //                     // } else if (desiredLength < Constants.ArmConstants.ArmMinLength) {
        //                     //     desiredLength = Constants.ArmConstants.ArmMinLength;
        //                     // }
        //                     // armSub.setLength(desiredLength, Constants.ArmConstants.LengthVel, Constants.ArmConstants.LengthAccelTime);

                            
        //                     if (desiredLength*Math.cos(desiredAngle) <= 0) {
        //                         desiredLength = currentLength;
        //                     } 
        //                 },
        //                 armSub));
                            // if (desiredLength * Math.cos(desiredAngle) <= 0) {
                            //     desiredLength = currentLength;
                            // }

                            // if (desiredAngle > Constants.ArmConstants.ArmMaxAngle) {
                            //     desiredAngle = Constants.ArmConstants.ArmMaxAngle;
                            // } else if (desiredAngle < Constants.ArmConstants.ArmMinAngle) {
                            //     desiredAngle = Constants.ArmConstants.ArmMinAngle;
                            // }
                            // wpk commented out until after gb repaired
                            // armSub.setAngle(desiredAngle, Constants.ArmConstants.AngleVel,
                            // Constants.ArmConstants.AngleAccelerationTime);  
    }
    
    private void configureButtonBindings() {

        driverController = new Joystick(1);
        operatorButtonController = new Joystick(2);
        operatorAxisController = new Joystick(3);

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);

        // xAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        // yawAxis = Constants.LogitechDualActionConstants.RightJoystickX;
        xAxis = Constants.RadioMasterConstants.LeftGimbalY;
        yawAxis = Constants.RadioMasterConstants.RightGimbalX;
        angleAxis = Constants.LogitechDualActionConstants.LeftJoystickY;
        extensionAxis = Constants.LogitechDualActionConstants.RightJoystickX;

        driverToggleClawButton = new JoystickButton(driverController, RadioMasterConstants.ButtonA);
        driverToggleClawButton.onTrue(toggleClaw);

        operatorToggleClawButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonY);
        operatorToggleClawButton.onTrue(toggleClaw);


        /* * * * * * ARM BUTTONS * * * * * */

        moveToRestingPositionButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonA);
        moveToRestingPositionButton.onTrue( new ArmPathGenerator(Arm.Position.Resting, armSub));

        moveToBottomButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftStick);
        moveToBottomButton.onTrue( new ArmPathGenerator(Arm.Position.Bottom, armSub));

        moveToMiddleButton = new JoystickButton(operatorButtonController, XboxControllerConstants.LeftBumper);
        moveToMiddleButton.onTrue( new ArmPathGenerator(Arm.Position.Middle, armSub));

        moveToTopButton = new JoystickButton(operatorButtonController, XboxControllerConstants.ButtonX);
        moveToTopButton.onTrue( new ArmPathGenerator(Arm.Position.Top, armSub));

        moveToFloorButton = new JoystickButton(operatorButtonController, XboxControllerConstants.RightBumper);
        moveToFloorButton.onTrue( new ArmPathGenerator(Arm.Position.Floor, armSub));

        moveToPlayerStationButton = new JoystickButton(operatorButtonController, XboxControllerConstants.RightStick);
        moveToPlayerStationButton.onTrue( new ArmPathGenerator(Arm.Position.PlayerStation, armSub));
               
        /* * * * * * VISION BUTTONS * * * * * */

        alignWithAprilTagsButton = new JoystickButton(driverController, 6);
        alignWithAprilTagsButton.onTrue(new AlignWithAprilTags(visionSub, driveSub));

        alignWithGamePieceButton = new JoystickButton(driverController, RadioMasterConstants.ButtonD);
        alignWithGamePieceButton.onTrue(new AlignWithGamePiece(visionSub, driveSub));

        alignWithPoleButton = new JoystickButton(driverController, 8);
        alignWithPoleButton.onTrue(new AlignWithPole(visionSub, driveSub));
    }

//     public Command getAutonomousCommand() {
//         HashMap<String, Command> eventMap = new HashMap<>();

//         eventMap.put("MoveToTop", new PrintCommand("\t\t\t*** PASSED FIRST LEG ***"));
//         eventMap.put("OpenClaw", new PrintCommand("\t\t\t*** HALF WAY THERE (living on a prayer) ***"));
//         eventMap.put("MoveToResting", new PrintCommand("\t\t\t*** ALMOST, I SWEAR ***"));

//         // eventMap.put("MoveToTop", new ArmPathGenerator(Arm.Position.Top, armSub));
//         // eventMap.put("OpenClaw", new ToggleClaw(clawSub));
//         // eventMap.put("MoveToResting", new ArmPathGenerator(Arm.Position.Resting, armSub));

// //        PathPlannerTrajectory traj = PathPlanner.loadPath(autoPath, new PathConstraints(1, 4));
//         PathPlannerTrajectory traj = PathPlanner.loadPath("Charging_Station_Only", new PathConstraints(1, 4), true);



//         RamseteController controller = new RamseteController();

//         Command ic = new InstantCommand(() -> {
//             // driveSub.resetEncoders();
//             driveSub.resetOdometry(traj.getInitialPose());
//         });

//         Command pathFollowingCommand = new PPRamseteCommand(
//                 traj,
//                 driveSub::getPose,
//                 controller,
//                 new DifferentialDriveKinematics(0.75),
//                 driveSub::setSpeeds,
//                 true,
//                 driveSub);

//         Command followPathWithEvents = new FollowPathWithEvents(pathFollowingCommand, traj.getMarkers(),
//                 eventMap);

//         // return new SequentialCommandGroup(ic, pathFollowingCommand);
//         return new SequentialCommandGroup(ic, followPathWithEvents);
//     }


public Command getAutonomousCommand() {

    SequentialCommandGroup g = new SequentialCommandGroup() ;

    g.addCommands(
        new ArmPathGenerator(Arm.Position.Bottom, armSub),
        new OpenClaw(clawSub),
        new ArmPathGenerator(Arm.Position.Resting, armSub)
    );

    g.addRequirements(armSub, clawSub);

    return null;
}


    public String getAutoPath() {
        return this.autoPath ;
    }

    public void setAutoPath( String p) {
        this.autoPath = p ;
    }
}