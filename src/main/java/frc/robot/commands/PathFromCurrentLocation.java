package frc.robot.commands;

import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drive;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.* ;

/** Custom PathPlanner version of RamseteCommand */
public class PathFromCurrentLocation extends CommandBase {
  private final Timer timer = new Timer();
  private final boolean usePID;
  private final PathConstraints constraints ;
  private PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final RamseteController controller;
  private final SimpleMotorFeedforward feedforward;
  private final DifferentialDriveKinematics kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
  private final PIDController leftController;
  private final PIDController rightController;
  private final BiConsumer<Double, Double> output;
  private final boolean useAllianceColor;
  private final boolean reversed ;
  private final Drive driveSub ;

  private List<PathPoint> points ;

  private DifferentialDriveWheelSpeeds prevSpeeds;
  private double prevTime;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError = PathFromCurrentLocation::defaultLogError;

  private static final double FIELD_WIDTH_METERS = 8.02;  



  /**
   * Constructs a new PPRamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
   * the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param controller The RAMSETE follower used to follow the trajectory.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and right wheel speeds.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public PathFromCurrentLocation(
      List<PathPoint> pathPoints ,
      PathConstraints constraints ,
      boolean reversed ,
      // PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      RamseteController controller,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      boolean useAllianceColor,
      Drive driveSub ,
      Subsystem... requirements
      ) {

    this.poseSupplier = poseSupplier;
    this.controller = controller;
    this.kinematics = kinematics;
    this.output = outputMetersPerSecond;

    this.feedforward = null;
    this.speedsSupplier = null;
    this.leftController = null;
    this.rightController = null;
    this.useAllianceColor = useAllianceColor;

    this.usePID = false;

    this.points = pathPoints ;
    this.driveSub = driveSub ;
    this.constraints = constraints ;
    this.reversed = reversed ;

    addRequirements(requirements);

    // List<EventMarker> markers = getMarkersFromJson(json);    

    // if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
    //   DriverStation.reportWarning(
    //       "You have constructed a path following command that will automatically transform path states depending"
    //           + " on the alliance color, however, it appears this path was created on the red side of the field"
    //           + " instead of the blue side. This is likely an error.",
    //       false);
    // }
  }


  @Override
  public void initialize() {

    // Need to translate the current position based on alliance
    if ( useAllianceColor && DriverStation.getAlliance() == Alliance.Red) {
        Translation2d transformedTranslation = new Translation2d(driveSub.getPose().getX(), FIELD_WIDTH_METERS - driveSub.getPose().getY());
        // Rotation2d transformedHeading = driveSub.getPose().getRotation().plus(new Rotation2d(Math.PI)).times(-1);
//        Rotation2d transformedHeading = driveSub.getPose().getRotation().times(-1);
        Rotation2d transformedHeading = driveSub.getPose().getRotation();
        points.add(0, new PathPoint(transformedTranslation, transformedHeading));
    } else {
//        points.add(0, new PathPoint(driveSub.getPose().getTranslation(), driveSub.getHeading()));
        points.add(0, new PathPoint(driveSub.getPose().getTranslation(), driveSub.getHeading().plus(new Rotation2d(Math.PI))));
    }

    System.out.println("points before generating") ;
    for ( var p : points) {
      System.out.println("position " + p.position + ", heading " + p.heading) ;
    }

    trajectory  = PathPlanner.generatePath ( constraints, reversed, points ) ;

    if (useAllianceColor ) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    this.prevTime = -1;

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    PathPlannerTrajectory.PathPlannerState initialState = transformedTrajectory.getInitialState();

    this.prevSpeeds =
        this.kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

    this.timer.reset();
    this.timer.start();

    if (this.usePID) {
      this.leftController.reset();
      this.rightController.reset();
    }

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    double dt = currentTime - this.prevTime;

    if (this.prevTime < 0) {
      this.prevTime = currentTime;
      return;
    }

    Pose2d currentPose = this.poseSupplier.get();
    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);

    PathPlannerServer.sendPathFollowingData(desiredState.poseMeters, currentPose);

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        this.kinematics.toWheelSpeeds(targetChassisSpeeds);

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (this.usePID) {
      double leftFeedforward =
          this.feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - this.prevSpeeds.leftMetersPerSecond) / dt);
      double rightFeedforward =
          this.feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - this.prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + this.leftController.calculate(
                  this.speedsSupplier.get().leftMetersPerSecond, leftSpeedSetpoint);
      rightOutput =
          rightFeedforward
              + this.rightController.calculate(
                  this.speedsSupplier.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    this.output.accept(leftOutput, rightOutput);
    this.prevSpeeds = targetWheelSpeeds;
    this.prevTime = currentTime;

    if (logTargetPose != null) {
      logTargetPose.accept(desiredState.poseMeters);
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.poseMeters.getRotation()));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
      this.output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPRamseteCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPRamseteCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber("PPRamseteCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPRamseteCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError
  ) {
    PathFromCurrentLocation.logActiveTrajectory = logActiveTrajectory;
    PathFromCurrentLocation.logTargetPose = logTargetPose;
    PathFromCurrentLocation.logSetpoint = logSetpoint;
    PathFromCurrentLocation.logError = logError;
  }
}
