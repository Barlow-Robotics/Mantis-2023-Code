// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class MoveToArmLength extends CommandBase {
  /** Creates a new MoveToRestingPosition. */

  Arm armSub;
  Claw clawSub;

  double armLength;
  
  public MoveToArmLength(Arm a, Claw c, double armL) {
    
      armSub = a;
      clawSub = c;
      armLength = armL;

      addRequirements(armSub, clawSub);
    }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setArmLength(armLength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armSub.getArmLength() - armLength) < Constants.ArmConstants.armLengthTolerance;
}

}