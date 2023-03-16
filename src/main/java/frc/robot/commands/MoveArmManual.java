// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Arm;

// public class MoveArmManual extends CommandBase {
//   /** Creates a new MoveArmManual. */

//   Arm armSub;

//   public MoveArmManual(Arm a) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     armSub = a;
//     addRequirements(armsub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     /* Angle */
//         double currentAngle = armSub.getAngle();
//         double desiredAngle = currentAngle +
//         (operatorButtonController.getRawAxis(angleAxis) * Constants.ArmConstants.AngleMultiplier);

//         if (desiredAngle > ArmConstants.ArmMaxAngle) {
//         desiredAngle = ArmConstants.ArmMaxAngle;
//         } else if (desiredAngle < ArmConstants.ArmMinAngle) {
//         desiredAngle = Constants.ArmConstants.ArmMinAngle;
//         }

//         armSub.setAngle(desiredAngle, ArmConstants.AngleVel,
//         Constants.ArmConstants.AngleAccelerationTime);

//         /* Extension */
//         double currentLength = armSub.getLength();
//         double desiredLength = currentLength + 
//         operatorButtonController.getRawAxis(extensionAxis) * Constants.ArmConstants.LengthMultiplier;

//         if (desiredLength > ArmConstants.ArmMaxLength) {
//         desiredLength = ArmConstants.ArmMaxLength;
//         } else if (desiredLength < ArmConstants.ArmMinLength) {
//         desiredLength = ArmConstants.ArmMinLength;
//         }

//         armSub.setLength(desiredLength, ArmConstants.LengthVel,
//         ArmConstants.LengthAccelTime);

//         // if (desiredLength * Math.cos(desiredAngle) <= 0) {
//         // desiredLength = currentLength;
//         // }
//         // double desiredAngle = currentAngle +
//         operatorButtonController.getRawAxis(1) *
//         Constants.ArmConstants.AngleMultiplier;

//         // if (desiredAngle > Constants.ArmConstants.ArmMaxAngle) {
//         // desiredAngle = Constants.ArmConstants.ArmMaxAngle;
//         // } else if (desiredAngle < Constants.ArmConstants.ArmMinAngle) {
//         // desiredAngle = Constants.ArmConstants.ArmMinAngle;
//         // }

//         // /* Extension */
//         // double currentLength = armSub.getLength();
//         // double desiredLength = currentLength +
//         operatorButtonController.getRawAxis(2) *
//         Constants.ArmConstants.LengthMultiplier;

//         // if (desiredLength > Constants.ArmConstants.ArmMaxLength) {
//         // desiredLength = Constants.ArmConstants.ArmMaxLength;
//         // } else if (desiredLength < Constants.ArmConstants.ArmMinLength) {
//         // desiredLength = Constants.ArmConstants.ArmMinLength;
//         // }
//         // armSub.setLength(desiredLength, Constants.ArmConstants.LengthVel,
//         Constants.ArmConstants.LengthAccelTime);

//         if (desiredLength*Math.cos(desiredAngle) <= 0) {
//         desiredLength = currentLength;
//         }
//         },
//         armSub));
//         if (desiredLength * Math.cos(desiredAngle) <= 0) {
//         desiredLength = currentLength;
//         }

//         if (desiredAngle > Constants.ArmConstants.ArmMaxAngle) {
//         desiredAngle = Constants.ArmConstants.ArmMaxAngle;
//         } else if (desiredAngle < Constants.ArmConstants.ArmMinAngle) {
//         desiredAngle = Constants.ArmConstants.ArmMinAngle;
//         }
//         wpk commented out until after gb repaired
//         armSub.setAngle(desiredAngle, Constants.ArmConstants.AngleVel,
//         Constants.ArmConstants.AngleAccelerationTime);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
