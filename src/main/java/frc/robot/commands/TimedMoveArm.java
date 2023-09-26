// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;

public class TimedMoveArm extends CommandBase {

    Arm armSub;

    double angle;
    double angleVelocity;
    double angleAccelerationTime;
    long rotateStartTime ;

    double length;
    double extensionVelocity;
    double extensionAcceleration;
    long extendStartTime ;
    Position state;

    long commandStartTime ;


    public TimedMoveArm(
            Arm a,
            double angle,
            double rotationVelocity,
            double rotationAcceleration,
            long rotateStartMillis ,
            double length,
            double extensionVelocity,
            double extensionAcceleration,
            long extendStartMillis ,
            Position state
            ) {

        this.armSub = a;
        this.angle = angle;
        this.angleVelocity = rotationVelocity;
        this.angleAccelerationTime = rotationAcceleration;
        this.rotateStartTime = rotateStartMillis ;
        this.length = length;
        this.extensionVelocity = extensionVelocity;
        this.extensionAcceleration = extensionAcceleration;
        this.extendStartTime = extendStartMillis ;
        this.state = state;

        addRequirements(armSub);
    }

    @Override
    public void initialize() {

        commandStartTime = System.currentTimeMillis() ;

        armSub.setState(Arm.Position.Transition);
        armSub.setAngle(angle, angleVelocity, angleAccelerationTime);
        armSub.setLength(length, extensionVelocity, extensionAcceleration);
        System.out.format("started move command: angle %5.2f, length %5.2f%n", angle, length) ;

    }

    @Override
    public void execute() {
        if ( System.currentTimeMillis() >= commandStartTime + rotateStartTime) {
            armSub.setAngle(angle, angleVelocity, angleAccelerationTime);
        }

        if ( System.currentTimeMillis() >= commandStartTime + extendStartTime) {
            armSub.setLength(length, extensionVelocity, extensionAcceleration);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            armSub.stopMoving();
            System.out.println("interrupted move command") ;
        }
        System.out.format("finished move command: angle %5.2f, length %5.2f%n", angle, length) ;
    }

    @Override
    public boolean isFinished() {
        if (
           ( angleVelocity == 0.0 || Math.abs(armSub.getAngle() - angle) <= Constants.ArmConstants.ArmAngleTolerance )
           && ( extensionVelocity == 0.0 ||  Math.abs(armSub.getLength() - length) <= Constants.ArmConstants.ArmLengthTolerance)
         ) {
            armSub.setState(state);
            return true;
        } else {
            return false;
        }
    }
}