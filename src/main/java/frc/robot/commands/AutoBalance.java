// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {

    PIDController controller = new PIDController(
        Constants.DriveConstants.AutoBalanceKP, 
        0.0, 
        Constants.DriveConstants.AutoBalanceKP * 0.7) ;

    private Drive driveSub;
    double ff = 0.0;
    double previousError;

    public AutoBalance(Drive d) {
        driveSub = d;
        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        controller.reset() ;
    }

    @Override
    public void execute() {

        double error = driveSub.getPitch();
        if (Math.abs(driveSub.getPitch()) < 2.5) {
            error = 0;
        } else if (Math.abs(driveSub.getPitch()) > 14) {
            error = 14 * Math.signum(error);
        }
        var output = controller.calculate(error) ;

        NetworkTableInstance.getDefault().getEntry("autobalance/error").setDouble(error);
        NetworkTableInstance.getDefault().getEntry("autobalance/output_speed").setDouble(output);

        driveSub.setSpeeds(output, output);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getFF() {
        return this.ff;
    }

    public void setFF(double value) {
        ff = value;
    }

    // public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty("Feed Forward", this::getFF, this::setFF);
    // }

}