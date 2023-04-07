// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CalibrateArmExtention;
import frc.robot.commands.CalibrateArmRotations;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Arm;

/* The VM is configured to automatically run this class, and to call the functions
corresponding to each mode, as described in the TimedRobot documentation. If you change
the name of this class or the package after creating this project, you must also update
the build.gradle file in the project. */
public class Robot extends TimedRobot {
    // public final Arm armSub = new Arm();

    private Command autonomousCommand;

    private Field2d field ;


    private RobotContainer robotContainer;

    TalonFXConfiguration config = new TalonFXConfiguration(); // factory default settings // O

    private boolean calibrationPerformed = false;

    static long startTime = System.currentTimeMillis() ;

    static HashMap<Command, Long> startTimes = new HashMap() ;

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
        robotContainer.armSub.stopMoving(); // Sets percent output of everything (rotate, extend, claw) to zero
        robotContainer.clawSub.stopMoving();
        robotContainer.driveSub.stopMoving () ;
        this.calibrationPerformed = false;
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
        robotContainer.driveSub.resetHeading();
        robotContainer.armSub.stopMoving(); // Need to figure out how to set percent output of everything (rotate,
                                            // extend,
        // claw) to zero

        SequentialCommandGroup cg = new SequentialCommandGroup();

        if (!calibrationPerformed  && !Robot.isSimulation()) {
            Command calibrateRotation = new CalibrateArmRotations(robotContainer.armSub);
            Command calibrateLength = new CalibrateArmExtention(robotContainer.armSub);
            Command setState = new InstantCommand(() -> robotContainer.armSub.setState(Arm.Position.Home));

            cg.addCommands(
                    calibrateLength,
                    calibrateRotation,
                    setState,
                    new InstantCommand(() -> this.calibrationPerformed = true),
                    new PrintCommand("Calibration Complete"));
        }

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        autonomousCommand = robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            cg.addCommands(autonomousCommand);
        }
        cg.schedule();

        
    }

    // /** This function is called periodically during autonomous. */
    // @Override
    public void autonomousPeriodic() {
        int wpk = 1 ;
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running.
        // If you want the autonomous to continue until interrupted by another command,
        // remove this line or comment it out.
        robotContainer.armSub.stopMoving(); // Need to figure out how to set percent output of everything (rotate,
                                            // extend,
        // claw) to zero
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        if (!calibrationPerformed && !Robot.isSimulation()) {
            Command calibrateRotation = new CalibrateArmRotations(robotContainer.armSub);
            Command calibrateLength = new CalibrateArmExtention(robotContainer.armSub);
            // wpk might want to change this to a move command.
            Command setArmLength = new InstantCommand(
                    () -> robotContainer.armSub.setLength(0.0, Constants.ArmConstants.ExtendVel,
                            Constants.ArmConstants.ExtendAccel));
            Command setState = new InstantCommand(() -> robotContainer.armSub.setState(Arm.Position.Home));

            SequentialCommandGroup calbrationSequence = new SequentialCommandGroup(
                    calibrateLength,
                    calibrateRotation,
                    setArmLength,
                    setState,
                    new InstantCommand(() -> {
                        robotContainer.armSub.setState(Arm.Position.Home);
                        this.calibrationPerformed = true;
                    }),
                    new PrintCommand("Calibration Complete"));
            calbrationSequence.schedule();
        }

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        robotContainer.armSub.stopMoving(); // Need to figure out how to set percent output of everything (rotate,
                                            // extend,
        // claw) to zero
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
        robotContainer.armSub.simulationInit();
        robotContainer.driveSub.simulationInit();
        robotContainer.clawSub.simulationInit();
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    static public void reportCommandStart(Command c) {
        double deltaTime = ((double)System.currentTimeMillis() - startTime) / 1000.0 ;
        System.out.println(deltaTime + ": Started " + c.getName())    ;     
        startTimes.putIfAbsent(c, System.currentTimeMillis() ) ;
    }

    static public void reportCommandFinish(Command c) {
        if ( startTimes.containsKey(c)) {
            long currentTime = System.currentTimeMillis() ;
            double deltaTime = ((double)currentTime - startTime) / 1000.0 ;
            double elapsedTime = (double)(currentTime - startTimes.get(c)) / 1000.0  ;
            System.out.println(deltaTime + ": Finished (elapsed time " + elapsedTime + ")" + c.getName()) ;     
            startTimes.remove(c) ;
        }
    }


    public void robotInit() {

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(true) ;

        field = new Field2d();
        SmartDashboard.putData("Field", field) ;

        CommandScheduler.getInstance().onCommandInitialize( Robot::reportCommandStart ) ;
        CommandScheduler.getInstance().onCommandFinish(Robot::reportCommandFinish);        
        CommandScheduler.getInstance().onCommandInterrupt( this::handleInterrupted) ;

    }

    @Override
    public void robotPeriodic() {

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.

        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(robotContainer.driveSub);
        SmartDashboard.putData(robotContainer.visionSub);
        SmartDashboard.putData(robotContainer.armSub);
        SmartDashboard.putData(robotContainer.clawSub);
        field.setRobotPose(robotContainer.driveSub.getPose());
            // Push the trajectory to Field2d.
        if (robotContainer.getCurrentTrajectory() != null) {
            field.getObject("traj").setTrajectory(robotContainer.getCurrentTrajectory());
        }

        if (this.isDisabled()) {
            robotContainer.armSub.stopMoving();
            robotContainer.clawSub.stopMoving();
        }
        CommandScheduler.getInstance().run();

    }

    private void handleInterrupted( Command c) {
        System.out.println("Commmand " + c + " named " + c.getName() + " was interrupted") ;
    }

}