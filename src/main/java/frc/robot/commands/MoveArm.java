// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class MoveArm extends CommandBase {

    Arm arm;

    double angle ;
    double angleVelocity ;
    double angleAcceleration ;
    double length ;
    double extensionVelocity ;
    double extensionAcceleration ;


    /** Creates a new MoveArm. */
    public MoveArm(
        Arm a, 
        double angle, 
        double angleVelocity, 
        double angleAcceleration, 
        double length, 
        double extensionVelocity, 
        double extensionAcceleration 
    ) {
        
        // wpk may want to validate angle and length against minimums and maximums
        // and override the provided values if necessary. e.g., if length > maxLength then length = maxLength...

        this.arm = a ;
        this.angle = angle ;
        this.angleVelocity = angleVelocity ;
        this.angleAcceleration = angleAcceleration ;
        this.length = length ;
        this.extensionVelocity = extensionVelocity ;
        this.extensionAcceleration = extensionAcceleration ;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm) ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setAngle(angle, angleVelocity, angleAcceleration);
        arm.setLength(length, extensionVelocity, extensionAcceleration);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stopMoving();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // wpk
        // need to return true if angle is at angle setpoint (withiin tolerance) and 
        // length is at length setpoint (with tolerance)

        return false;
    }
}
