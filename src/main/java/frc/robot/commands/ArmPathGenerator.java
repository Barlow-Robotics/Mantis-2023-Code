// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.Constants;

public class ArmPathGenerator extends CommandBase {
    Arm armSub;
    Arm.Position to;

    public ArmPathGenerator(Arm.Position to, Arm armSub) {
        this.to = to;
        this.armSub = armSub;
        // addRequirements(armSub);
    }

    /* * * * * * FROM HOME * * * * * */

    public SequentialCommandGroup getPathFromHome() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                break; // do nothing in this case since we're already at resting

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Transition:
                break; // Not applicable
        }

        g.setName("Path from Home to " + to.toString()) ;
        return g;
    }

    /* * * * * * FROM FLOOR * * * * * */

    public SequentialCommandGroup getPathFromFloor() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.HomeArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Floor:
                break; // Not applicable

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.PlayerStationArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Transition:
                break;// Not applicable
        }

        g.setName("Path from Floor to " + to.toString()) ;
        return g;
    }

    /* * * * * * FROM BOTTOM * * * * * */

    public SequentialCommandGroup getPathFromBottom() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.HomeArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Bottom:
                break; // Not applicable

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                0.0,
                                0.0,
                                to),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.PlayerStationArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Transition:
                break; // todo
        }

        return g;
    }

    /* * * * * * FROM MIDDLE * * * * * */

    public SequentialCommandGroup getPathFromMiddle() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.HomeArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Middle:
                break; // Not applicable

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.MiddleArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.PlayerStationArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Transition:
                break; // Not applicable
        }

        return g;
    }

    /* * * * * * FROM TOP * * * * * */

    public SequentialCommandGroup getPathFromTop() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.HomeArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.BottomArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.MiddleArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Top:
                break; // Not applicable

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.PlayerStationArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Transition:
                break;// todo
        }

        g.setName("Path from Top to " + to.toString()) ;
        return g;
    }

    /* * * * * * FROM PLAYER STATION * * * * * */

    private SequentialCommandGroup getPathFromPlayerStation() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Home:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.PlayerStationArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.HomeArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.HomeArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.HomeArmLength,
                                0.0,
                                0.0,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.RotateVel,
                                Constants.ArmConstants.RotateAccel,
                                Constants.ArmConstants.FloorArmLength,
                                0.0,
                                0.0,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                0.0,
                                0.0,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.ExtendVel,
                                Constants.ArmConstants.ExtendAccel,
                                to));
                break;

            case PlayerStation:
                break;

            case Transition:
                break; // Not Applicable
        }

        return g;
    }

    private SequentialCommandGroup getPathFromTransition() {

        SequentialCommandGroup g = new SequentialCommandGroup();

        // Moving from transition is a tricky case so we're using a simple, but
        // suboptimal approach
        // First, no matter where we are, we'll retract the arm all the way. After that,
        // we'll move
        // to the desured angle and then move to the desired length. This is a three
        // step approach
        // because if we're in transition, we don't realy know where we are.
        // In the future, maybe replace this with something smarter.

        Command first = new MoveArm(
                armSub,
                Constants.ArmConstants.MiddleArmAngle,
                0.0,
                0.0,
                armSub.getLength(),
                Constants.ArmConstants.ExtendVel,
                Constants.ArmConstants.ExtendAccel,
                Position.Transition).withTimeout(4.0);

//        g.addCommands(first);
        g.addCommands(getPathFromHome());
        g.setName("Path from transition to " + to ) ;

        return g;
    }

    @Override
    public void initialize() {

        SequentialCommandGroup g = null;

        switch (armSub.getState()) {
            case Home:
                g = getPathFromHome();
                break;
            case Bottom:
                g = getPathFromBottom();
                break;
            case Middle:
                g = getPathFromMiddle();
                break;
            case Top:
                g = getPathFromTop();
                break;
            case Floor:
                g = getPathFromFloor();
                break;
            case PlayerStation:
                g = getPathFromPlayerStation();
                break;
            case Transition:
                g = getPathFromTransition();
                break;
        }

        if (g != null) {
            g.schedule();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}