// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class Pivot extends CommandBase {

    Drive driveSub;
    double targetDistance;
    double velocity;
    double targetAngle;

    double startingLeftDistance;
    double startingRightDistance;


    /** Creates a new DriveDistance. */
    public Pivot(Drive d, double a, double s) {
        // Use addRequirements() here to declare subsystem dependencies.
        driveSub = d;
        targetAngle = a;
        velocity = s;
        addRequirements(driveSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingLeftDistance = driveSub.getLeftDistance() ;
        startingRightDistance = driveSub.getRightDistance() ;
        targetDistance = Constants.DriveConstants.circumferenceWithTrackWidth * ( targetAngle / 360.0 ) ;
        System.out.println("Starting heading before pivot " + driveSub.getHeading()) ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSub.setSpeeds(-Math.signum(targetAngle)*velocity, Math.signum(targetAngle)* velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSub.setSpeeds(0.0, 0.0);
        System.out.println("   heading after pivot " + driveSub.getHeading()) ;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double distanceTraveled = Math.abs(driveSub.getLeftDistance() - startingLeftDistance);
        if (Math.abs(distanceTraveled) >= Math.abs(targetDistance)) {
            return true ;
        }
        return false;
    }
}