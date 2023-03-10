// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.sim.PhysicsSim;

public class Claw extends SubsystemBase {
    /** Creates a new Claw. */

    WPI_TalonFX clawMotor; // For adjusting angle

    //Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    Solenoid closeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.CloseSolenoidID);
    Solenoid openSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
            Constants.ClawConstants.OpenSolenoidID);

    TimeOfFlight distanceSensor = new TimeOfFlight(Constants.ClawConstants.DistanceSensorID);

    // Add distance sensor from playing with fusion

    Arm armSub;
    // private final Timer timer = new Timer();
    
    boolean autoCloseEnabled = true;
    boolean open = false ;

    public Claw(Arm a) { // add arm to constructors
        armSub = a;
        clawMotor = new WPI_TalonFX(Constants.ClawConstants.ClawMotorID); // needs config

        setClawMotorConfig(clawMotor);
        clawMotor.setSelectedSensorPosition(0) ;

        this.close() ;
    }

    private void setClawMotorConfig(WPI_TalonFX motor) { 
        motor.configClosedloopRamp(Constants.ClawConstants.ClawClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ClawConstants.ClawManualVoltageRampingConstant);
        motor.config_kF(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKF);
        motor.config_kP(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKP);
        motor.config_kI(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKI);
        motor.config_kD(Constants.ClawConstants.ClawPID_id, Constants.ClawConstants.ClawKD);

        motor.setInverted(TalonFXInvertType.Clockwise);

        motor.configMotionCruiseVelocity(Constants.ClawConstants.ClawSpeed * Constants.ClawConstants.DegreesPerSecToCountsPer100MSec );
        motor.configMotionAcceleration( Constants.ClawConstants.DegreesPerSecToCountsPer100MSec / 0.1);  // wpk add a constant for this one

        motor.configMotionSCurveStrength(Constants.ClawConstants.AccelerationSmoothing);


        // wpk add something for soft limits.

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    @Override
    public void periodic() {
        // 0.0 is perpendicular to arm bar
        setAngle( -armSub.getAngle()); 
        
        // if (isOpen() && autoCloseEnabled && distanceSensor.getRange() <= (ClawConstants.InchesForAutoClosing) * Constants.InchesToMillimeters) {
        //     close();
        //     disableAutoClose();
        // }
        // else if (isOpen() && distanceSensor.getRange() >= (ClawConstants.ClawLengthInches) * Constants.InchesToMillimeters) {
        //     enableAutoClose();
        // }

        NetworkTableInstance.getDefault().getEntry("claw/actualAngle").setDouble(this.getAngle()) ;
        NetworkTableInstance.getDefault().getEntry("claw/isOpen").setBoolean(this.isOpen()) ;

    }

    public double getAngle() {
        double result = clawMotor.getSelectedSensorPosition() / Constants.ClawConstants.CountsPerClawDegree;
        return result;
    }

    public void setAngle(double desiredAngle) {
        double setAngle = desiredAngle * ClawConstants.CountsPerClawDegree; 
//        clawMotor.set(TalonFXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, Constants.ClawConstants.ff );
        clawMotor.set(TalonFXControlMode.Position, setAngle );
        NetworkTableInstance.getDefault().getEntry("claw/desiredAngle").setDouble(desiredAngle) ;
    }


    public void stopMoving() {
        clawMotor.set( TalonFXControlMode.PercentOutput, 0.0) ;
    }

    public void open() {
        openSolenoid.set(true);
        closeSolenoid.set(false);
        open = true ;
    }

    public void close() {
        openSolenoid.set(false);
        closeSolenoid.set(true);
        open = false ;
    }

    public boolean isOpen() {
        return open;
    }

    public void enableAutoClose() {
        autoCloseEnabled = true;
    }

    public void disableAutoClose() {
        autoCloseEnabled = false;
    }



    // Simulation Support

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(clawMotor, 0.1, 6800 );
    }

}