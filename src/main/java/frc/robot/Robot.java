// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings // O

    Joystick opp = new Joystick(0);

    int state = 0; // O

    BufferedTrajectoryPointStream bufferedStream = new BufferedTrajectoryPointStream(); // O

    public boolean currentProfileButton;

    /*
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */

    /*
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    // @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    // /** This function is called periodically during autonomous. */
    // @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }

    public void robotInit() {

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        /* fill our buffer object with the excel points */
        initBuffer(MotionProfile.Points, MotionProfile.kNumPoints);

        
    }

    public void robotPeriodic(int current) {

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        // /* get joystick button and stick */
        // // boolean fireMotionProfile = _joy.getRawButton(2);
        // // double axis = _joy.getRawAxis(2);
       
        boolean moveToBottomRowProfile = robotContainer.operatorController.getRawButton(1); 
        boolean moveToMiddleRowProfile = robotContainer.operatorController.getRawButton(2);
        boolean moveToTopRowProfile = robotContainer.operatorController.getRawButton(3);
        boolean moveToRestProfile = robotContainer.operatorController.getRawButton(4);
        boolean moveToPlayerStationProfile = robotContainer.operatorController.getRawButton(5);
        boolean pickUpFromGroundProfile = robotContainer.operatorController.getRawButton(6);

        if (current == 1) { currentProfileButton = moveToBottomRowProfile; }
        else if (current == 2) { currentProfileButton = moveToMiddleRowProfile; }
        else if (current == 3) { currentProfileButton = moveToTopRowProfile; }
        else if (current == 4) { currentProfileButton = moveToRestProfile; }
        else if (current == 5) { currentProfileButton = moveToPlayerStationProfile; }
        else if (current == 6) { currentProfileButton = pickUpFromGroundProfile; }
        
        // /* if button is up, just drive the motor in PercentOutput */
        // if (moveToBottomRowProfile == false) {
        //     state = 0;
        // }

        // switch (state) {
        //     /* drive master talon normally */
        //     case 0:
        //         master.set(TalonFXControlMode.Velocity, ArmConstants.armRotateSpeed);
        //         if (moveToBottomRowProfile == true) {
        //             /* go to MP logic */
        //             state = 1;
        //         } 
        //         break;

        //     /* fire the MP, and stop calling set() since that will cancel the MP */
        //     case 1:
        //         /* wait for 10 points to buffer in firmware, then transition to MP */
        //         master.startMotionProfile(_bufferedStream, 10, TalonFXControlMode.MotionProfile.toControlMode());
        //         state = 2;
        //         // Instrum.printLine("MP started");
        //         break;

        //     /* wait for MP to finish */
        //     case 2:
        //         if (master.isMotionProfileFinished()) {
        //             // Instrum.printLine("MP finished");
        //             state = 3;
        //         }
        //         break;
        //     case 3:
        //         if (true) {

        //         }
        //         break;
        //     case 4:
        //         if() {

        //         }
        //         break;
        //     case 5:
        //         if() {

        //         }
        //         break;
        //     /* MP is finished, nothing to do */
        //     case 6:
        //         break;
        // }

        // /* print MP values */
        // // Instrum.loop(bPrintValues, _master);
    }

    /**
     * Fill _bufferedStream with points from csv/generated-table.
     *
     * @param profile  generated array from excel
     * @param totalCnt num points in profile
     */
    private void initBuffer(double[][] profile, int totalCnt) {

        boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).

        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                       // automatically, you can alloc just one

        /* clear the buffer, in case it was used elsewhere */
        bufferedStream.Clear();

        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {

            double direction = forward ? +1 : -1;
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            int durationMilliseconds = (int) profile[i][2];

            /* for each point, fill our structure and pass it to API */
            point.timeDur = durationMilliseconds;
            point.position = direction * positionRot * Constants.DriveConstants.countsPerRevolution; // Convert
                                                                                                     // Revolutions to
            // Units
            point.velocity = direction * velocityRPM * Constants.DriveConstants.countsPerRevolution / 600.0; // Convert
                                                                                                             // RPM to
            // Units/100ms
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (i == 0); /* set this to true on the first point */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            bufferedStream.Write(point);
        }
    }

}
