// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a differential drive style drivetrain. */
public class Drive extends SubsystemBase {
    WPI_TalonFX driveMotorLeftLeader;
    WPI_TalonFX driveMotorLeftFollower;
    WPI_TalonFX driveMotorRightLeader;
    WPI_TalonFX driveMotorRightFollower;

    DifferentialDrive diffDrive;

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);
    private final DifferentialDriveOdometry odometry;

    boolean simulationInitialized = false;

    // Gains are for example purposes only - must be determined for your own robot!
    // private final SimpleMotorFeedforward m_feedforward = new
    // SimpleMotorFeedforward(1, 3);

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the
     * gyro.
     */
    public Drive() {

        driveMotorLeftLeader = new WPI_TalonFX(Constants.DriveConstants.driveMotorLeftLeaderID);
        driveMotorLeftFollower = new WPI_TalonFX(Constants.DriveConstants.driveMotorLeftFollowerID);
        driveMotorRightLeader = new WPI_TalonFX(Constants.DriveConstants.driveMotorRightLeaderID);
        driveMotorRightFollower = new WPI_TalonFX(Constants.DriveConstants.driveMotorRightFollowerID);

        // Config Motors
        driveMotorLeftLeader.configFactoryDefault();
        driveMotorRightLeader.configFactoryDefault();
        driveMotorLeftFollower.configFactoryDefault();
        driveMotorRightFollower.configFactoryDefault();
        driveMotorLeftFollower.follow(driveMotorLeftLeader);
        driveMotorRightFollower.follow(driveMotorRightLeader);
        driveMotorRightLeader.setInverted(InvertType.InvertMotorOutput);
        driveMotorRightFollower.setInverted(InvertType.FollowMaster);
        driveMotorLeftFollower.setInverted(InvertType.FollowMaster);

        driveMotorLeftLeader.setSensorPhase(true);
        driveMotorRightLeader.setSensorPhase(true);

        diffDrive = new DifferentialDrive(driveMotorLeftLeader, driveMotorRightLeader);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), getPose());

        setDefaultNeutralMode();
        gyro.reset();
        CreateNetworkTableEntries();
    }

    public void periodic() {
        // NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(getLeftDistance());
        // NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(getRightDistance());
        // NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(getLeftSpeed());
        // NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(getRightSpeed());
        // NetworkTableInstance.getDefault().getEntry("drive/gyro_heading").setDouble(getGyroHeading());
        // NetworkTableInstance.getDefault().getEntry("drive/odometry/X").setDouble(m_odometry.getPoseMeters().getX());
        // NetworkTableInstance.getDefault().getEntry("drive/odometry/Y").setDouble(m_odometry.getPoseMeters().getY());
        // NetworkTableInstance.getDefault().getEntry("drive/odometry/theta")
        //         .setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());

        // Update the odometry in the periodic block
        odometry.update(
                gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());
    }

    public void setDefaultNeutralMode() {
        driveMotorLeftLeader.setNeutralMode(NeutralMode.Brake);
        driveMotorRightLeader.setNeutralMode(NeutralMode.Brake);
        // m_leftLeader.setNeutralMode(NeutralMode.Coast);
        // m_rightLeader.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {      //EP i never see this used anywhere, do we need it?
        driveMotorLeftLeader.set(Constants.DriveConstants.driveSpeed);
        driveMotorRightLeader.set(Constants.DriveConstants.driveSpeed);
    }

    private double getLeftSpeed() {     //EP i never see this used anywhere, do we need it?
        double s = driveMotorLeftLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.metersPerCount;
        return (s);
    }

    private double getRightSpeed() {    //EP i never see this used anywhere, do we need it?
        double s = -driveMotorRightLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.metersPerCount;
        return (s);
    }

    private double getLeftDistance() {
        double d = (driveMotorLeftLeader.getSelectedSensorPosition() / Constants.DriveConstants.countsPerRevolution)
                * Constants.DriveConstants.metersPerRevolution;
        return (d);
    }

    private double getRightDistance() {
        double d = (-driveMotorRightLeader.getSelectedSensorPosition() / Constants.DriveConstants.countsPerRevolution)
                * Constants.DriveConstants.metersPerRevolution;
        return (d);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed       Linear velocity in m/s.
     * @param rot          Angular velocity in rad/s.
     * @param squareInputs Decreases input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot, boolean squareInputs) {
        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(driveMotorLeftLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(driveMotorLeftLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(100.0);
        diffDrive.arcadeDrive(xSpeed, rot, squareInputs);
//        diffDrive.curvatureDrive(xSpeed, rot, true);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), getPose());
    }

    public void resetEncoders() {
        driveMotorLeftLeader.setSelectedSensorPosition(0);
        driveMotorRightLeader.setSelectedSensorPosition(0);
    }

    public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    }

    public double getGyroHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    public void resetHeading() {
        gyro.reset();
    }

    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/pose/rotation").setDouble(0.0);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(driveMotorLeftLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorLeftFollower, 0.75, 6800, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorRightLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorRightFollower, 0.75, 6800, false);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
        PhysicsSim.getInstance().run();

        double headingNoise = 0.0; // (Math.random() - 0.5) * 4.0 ;
        // gyroSim.setAngle(this.m_odometry.getPoseMeters().getRotation().getDegrees() +
        // headingNoise);
        gyroSim.setAngle(5.0);
        gyroSim.setRate(1.0);
        NetworkTableInstance.getDefault().getEntry("drive/gyro/getAngle").setDouble(gyro.getAngle());
    }
}
