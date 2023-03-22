// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.Vision;
// import frc.robot.commands.DriveRobot;

// public class DriveToGamePiece extends CommandBase {
//   /** Creates a new DriveToGamePiece. */

//   Drive driveSub;
//   Vision visionSub;

//   double targetDistance; // meters?
//   double speed; // meters per second?

//   double startingLeftDistance;
//   double startingRightDistance;



//   public DriveToGamePiece(Drive d, Vision v, double dist) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     driveSub = d;
//     visionSub = v;

//     targetDistance = dist;

//     addRequirements(driveSub, visionSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     driveSub.resetOdometry(new Pose2d());
//     startingLeftDistance = driveSub.getLeftDistance();
//     startingRightDistance = driveSub.getRightDistance();

//     DriveRobot.pid.reset();
//     missedFrames = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // drive forward at set speed
//     // if game piece is seen, turn while continuing to move forawrd

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
