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

  Arm armSub;
  Claw clawSub;

  public MoveToTopRow(Arm a, Claw c) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = a;
    clawSub = c;
    
    addRequirements(armSub, clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setArmAngle(Constants.ArmConstants.topRowArmAngle);
    armSub.setArmLength(Constants.ArmConstants.topRowArmLength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armSub.getArmAngle() - Constants.ArmConstants.topRowArmAngle) < Constants.ArmConstants.armAngleTolerance;
  }
}
