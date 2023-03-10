// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;

public class MoveArm extends CommandBase {

    Arm armSub;

    double angle;
    double angleVelocity;
    double angleAccelerationTime;
    double length;
    double extensionVelocity;
    double extensionAcceleration;
    Position state;

    /** Creates a new MoveArm. */
    public MoveArm(
            Arm a,
            double angle,
            double rotationVelocity,
            double rotationAcceleration,
            double length,
            double extensionVelocity,
            double extensionAcceleration,
            Position state
            ) {

        // wpk may want to validate angle and length against minimums and maximums
        // and override the provided values if necessary. e.g., if length > maxLength
        // then length = maxLength...

        this.armSub = a;
        this.angle = angle;
        this.angleVelocity = rotationVelocity;
        this.angleAccelerationTime = rotationAcceleration;
        this.length = length;
        this.extensionVelocity = extensionVelocity;
        this.extensionAcceleration = extensionAcceleration;
        this.state = state;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("***** move command to " + state + " started");

        armSub.setState(Arm.Position.Transition);
        armSub.setAngle(angle, angleVelocity, angleAccelerationTime);
        armSub.setLength(length, extensionVelocity, extensionAcceleration);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armSub.setAngle(angle, angleVelocity, angleAccelerationTime);
        armSub.setLength(length, extensionVelocity, extensionAcceleration);
        NetworkTableInstance.getDefault().getEntry("move_arm/angle").setDouble(angle) ;
        NetworkTableInstance.getDefault().getEntry("move_arm/length").setDouble(length) ;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            armSub.stopMoving();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (
           ( angleVelocity == 0.0 || Math.abs(armSub.getAngle() - angle) <= Constants.ArmConstants.ArmAngleTolerance )
           && ( extensionVelocity == 0.0 ||  Math.abs(armSub.getLength() - length) <= Constants.ArmConstants.ArmLengthTolerance)
         ) {
            armSub.setState(state);
            System.out.println("***** move command to " + state + " is finished");
            return true;
        } else {
            return false;
        }
    }
}
