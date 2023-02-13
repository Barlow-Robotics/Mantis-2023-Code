// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MoveToTopRow extends CommandBase {
    /** Creates a new MoveToTopRow. */

    Arm arm;

    // wpk - If claw is not needed by this command, then we should remove it.
    Claw claw;

    public MoveToTopRow(Arm a, Claw c) {
        // Use addRequirements() here to declare subsystem dependencies.
        arm = a;
        claw = c;

        addRequirements(arm, claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // wpk need to fill in the values here.
        arm.setArmAngle(0, 0, 0);
        arm.setArmLength(0, 0, 0);


        arm.startProfiles();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(arm
                .getArmAngle() /* subtract angle of last point in path */ ) < Constants.ArmConstants.armAngleTolerance;
    }
}
