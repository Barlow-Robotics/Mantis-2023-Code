// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
        this.armSub = armSub ;
        addRequirements(armSub);
    }


    private SequentialCommandGroup getPathFromResting() {
        SequentialCommandGroup g = null ;

        switch (armSub.getState() ) {
            // case Arm.Position.Floor :
            case Floor :
               g = new SequentialCommandGroup(
                   new MoveArm(
                       armSub, 
                       Constants.ArmConstants.FloorArmAngle, 
                       Constants.ArmConstants.AngleVel, 
                       Constants.ArmConstants.AngleAccelerationTime,                    
                       0.0, 
                       0.0, 
                       0.25, 
                       Position.Transition) ,
                    new MoveArm(
                        armSub, 
                        Constants.ArmConstants.FloorArmAngle, 
                        Constants.ArmConstants.AngleVel, 
                        Constants.ArmConstants.AngleAccelerationTime,                    
                        Constants.ArmConstants.FloorArmLength, 
                        Constants.ArmConstants.LengthVel, 
                        Constants.ArmConstants.LengthAccelTime, 
                        Position.Transition) 
                ) ;
                break ;

            case Bottom:
                g = new SequentialCommandGroup(
                    new MoveArm( armSub,40.0, Constants.ArmConstants.AngleVel,Constants.ArmConstants.AngleAccelerationTime,                    
                        0.0,0.0,0.25,Position.Transition) ,
                    new MoveArm(armSub, 40.0,Constants.ArmConstants.AngleVel,Constants.ArmConstants.AngleAccelerationTime,                    
                        0.0,0.0,0.25,Position.Transition) 
                ) ;
                break ;

            case Middle:
                g = new SequentialCommandGroup(
                    new MoveArm( armSub,40.0, Constants.ArmConstants.AngleVel,Constants.ArmConstants.AngleAccelerationTime,                    
                        0.0,0.0,0.25,Position.Transition) ,
                    new MoveArm(armSub, 40.0,Constants.ArmConstants.AngleVel,Constants.ArmConstants.AngleAccelerationTime,                    
                        0.0,0.0,0.25,Position.Transition) 
                ) ;
                break ;

            case Top :  // todo
            case PlayerStation:  // todo
            case Transition: // todo


        }

        return g ;
    }



    private SequentialCommandGroup getPathFromFloor() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }


    private SequentialCommandGroup getPathFromBottom() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }


    private SequentialCommandGroup getPathFromMiddle() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }



    private SequentialCommandGroup getPathFromTop() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }


    private SequentialCommandGroup getPathFromPlayerStation() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }



    private SequentialCommandGroup getPathFromTransition() {
        SequentialCommandGroup g = null ;

        // todo fill in switch statement

        return g ;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SequentialCommandGroup g = null;

        switch( armSub.getState()) {
            case Resting:
                g = getPathFromResting() ;
                break ;
            case Bottom:
                g = getPathFromBottom() ;
                break ;
            case Middle:
                g = getPathFromMiddle() ;
                break ;
            case Top:
                g = getPathFromTop() ;
                break ;
            case Floor:
                g = getPathFromFloor() ;
                break ;
            case PlayerStation:
                g = getPathFromPlayerStation() ;
                break ;
            case Transition:
                g = getPathFromTransition() ;
                break ;
        }

        if ( g != null ) {
            CommandScheduler.getInstance().schedule(g);
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
