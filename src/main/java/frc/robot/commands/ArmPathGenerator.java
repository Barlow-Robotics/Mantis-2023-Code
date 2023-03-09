// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.Constants;

public class ArmPathGenerator extends CommandBase {
    /** Creates a new CommandGenerator. */

    Arm armSub;
    Arm.Position to;

    public ArmPathGenerator(Arm.Position to, Arm armSub) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.to = to;
        this.armSub = armSub;
        addRequirements(armSub);
    }

    /* * * * * * FROM RESTING * * * * * */

    private SequentialCommandGroup getPathFromResting() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                break; // do nothing in this case since we're already at resting

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.NoMovementVel,
                                Constants.ArmConstants.NoMovementVel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.BottomArmLength,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.NoMovementVel,
                                Constants.ArmConstants.NoMovementVel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.MiddleArmLength,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.MiddleArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.NoMovementVel,
                                Constants.ArmConstants.NoMovementVel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.TopArmLength,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.NoMovementVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.FloorArmLength,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.BottomArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.PlayerStationArmLength,
                                0.0,
                                0.25,
                                Position.Transition));
                break;

            case Transition:
                break; // todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromFloor() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.RestingArmLength,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.RestingArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.RestingArmLength,
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                50,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                0.0,
                                Constants.ArmConstants.AngleAccel,
                                0.05 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                100.0,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                110.0,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.7 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Floor:
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                93,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Transition:
                break;// todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromBottom() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                break; // Brea

            case Bottom:

                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.05 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                100,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.TopArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                27.6 / Constants.InchesToMeters,
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                35,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                Constants.ArmConstants.FloorArmAngle,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                0.0,
                                Constants.ArmConstants.LengthAccel,
                                to));

                // "1) angle down (-->35 deg)
                // 2) no extension?"
                break; // todo

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                93,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Transition:
                break;// todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromMiddle() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                g.addCommands(
                        new MoveArm(armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(armSub,
                                0.0,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                40,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.MiddleArmLength / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                40,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Middle:
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                110,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                Constants.ArmConstants.MiddleArmLength / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                110,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                27.6 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                35,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                93,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Transition:
                break;// todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromTop() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                40,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.05,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                100,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.05,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Top:
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                93,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case PlayerStation:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                93,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Transition:
                break;// todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromPlayerStation() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                break;

            case Bottom:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                35,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Middle:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                90,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                90,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                15 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Top:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                110,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                armSub.getLength(),
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                110,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                27.6 / Constants.InchesToMeters,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case Floor:
                g.addCommands(
                        new MoveArm(
                                armSub,
                                armSub.getAngle(),
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                Position.Transition),
                        new MoveArm(
                                armSub,
                                35,
                                Constants.ArmConstants.AngleVel,
                                Constants.ArmConstants.AngleAccel,
                                0.3,
                                Constants.ArmConstants.LengthVel,
                                Constants.ArmConstants.LengthAccel,
                                to));
                break;

            case PlayerStation:
                break;

            case Transition:
                break;// todo
        }

        return g;
    }

    private SequentialCommandGroup getPathFromTransition() {
        SequentialCommandGroup g = new SequentialCommandGroup();

        switch (to) {
            case Resting:
                break;

            case Bottom:
                break; // todo

            case Middle:
                break; // todo

            case Top:
                break; // todo

            case Floor:
                break; // todo

            case PlayerStation:
                break; // todo

            case Transition:
                break;// todo
        }

        return g;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SequentialCommandGroup g = null;

        switch (armSub.getState()) {
            case Resting:
                g = getPathFromResting();
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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
