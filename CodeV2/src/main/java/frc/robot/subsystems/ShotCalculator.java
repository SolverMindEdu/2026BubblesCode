package frc.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShotCalculator extends SubsystemBase {
  private static ShotCalculator instance;
  private final CommandSwerveDrivetrain drive;

  // State tracking
  private double lastHoodAngle = Double.NaN;
  private ShootParameters latestParameters = null;

  public ShotCalculator(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  public static ShotCalculator getInstance(CommandSwerveDrivetrain drive) {
    if (instance == null) instance = new ShotCalculator(drive);
    return instance;
  }

  public static ShotCalculator getInstance() {
    return instance;
  }

  /**
   * Data record containing all calculated parameters for a shot.
   */
  public record ShootParameters(
      double initialDistance,
      double hoodAngle,
      double flywheelSpeed,
      double lookaheadDistance,
      double robotHeadingRadians, // Field-relative angle in radians
      boolean isPassing
  ) {}

  private static final InterpolatingTreeMap<Double, Rotation2d> passingShotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    //test Values
    passingShotHoodAngleMap.put(1.45, Rotation2d.fromDegrees(19.0));
    passingShotHoodAngleMap.put(5.0, Rotation2d.fromDegrees(35)); // distance, angle
    passingShotFlywheelSpeedMap.put(1.45, 20.0); //distance RPS
    passingShotFlywheelSpeedMap.put(5.0, 50.0); //distance RPS

    passingTimeOfFlightMap.put(1.64227, 0.93); //distance, time
  }

  // Interpolation maps for shot lookups
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    // these the real values :coolface:
    // shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(14)); // distance, angle
    // shotHoodAngleMap.put(3.0, Rotation2d.fromDegrees(14)); // distance, angle
    shotHoodAngleMap.put(1.9, Rotation2d.fromDegrees(0)); // distance, angle
    shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(4)); // distance, angle
    shotHoodAngleMap.put(3.0, Rotation2d.fromDegrees(8)); // distance, angle
    shotHoodAngleMap.put(3.3, Rotation2d.fromDegrees(9.5)); // distance, angle
    shotHoodAngleMap.put(3.5, Rotation2d.fromDegrees(10)); // distance, angle
    shotHoodAngleMap.put(3.9, Rotation2d.fromDegrees(12.5)); // distance, angle
    shotHoodAngleMap.put(4.4, Rotation2d.fromDegrees(15.0)); // distance, angle
    shotHoodAngleMap.put(5.0, Rotation2d.fromDegrees(18.0)); // distance, angle
    shotHoodAngleMap.put(5.4, Rotation2d.fromDegrees(26.0)); // distance, angle
    shotHoodAngleMap.put(5.5, Rotation2d.fromDegrees(27.0)); // distance, angle

    shotFlywheelSpeedMap.put(1.9, 51.5); //distance RPS
    shotFlywheelSpeedMap.put(2.5, 54.5); //distance RPS
    shotFlywheelSpeedMap.put(3.0, 57.0); //distance RPS
    shotFlywheelSpeedMap.put(3.3, 58.0); //distance RPS
    shotFlywheelSpeedMap.put(3.5, 59.0); //distance RPS
    shotFlywheelSpeedMap.put(3.9, 61.0); //distance RPS
    shotFlywheelSpeedMap.put(4.4, 65.0); //distance RPS
    shotFlywheelSpeedMap.put(5.0, 71.5); //distance RPS
    shotFlywheelSpeedMap.put(5.4, 77.0); //distance RPS
    shotFlywheelSpeedMap.put(5.5, 79.5); //distance RPS


    timeOfFlightMap.put(1.64227, 0.93); //distance,  time
  }

  @Override
  public void periodic() {
    // always compute fresh each loop
    clearShootingParameters();

    try {
      ShootParameters p = getParameters(); // uses this.drive, not singleton

      SmartDashboard.putNumber("Shot/InitialDistance", p.initialDistance());
      SmartDashboard.putNumber("Shot/LookaheadDistance", p.lookaheadDistance());
      SmartDashboard.putNumber("Shot/HoodAngleDeg", p.hoodAngle());
      SmartDashboard.putNumber("Shot/FlywheelRPS", p.flywheelSpeed());
      SmartDashboard.putNumber("Shot/HeadingDeg", Units.radiansToDegrees(p.robotHeadingRadians()));
      SmartDashboard.putBoolean("Shot/IsPassing", p.isPassing());
    } catch (Exception e) {
      SmartDashboard.putString("Shot/Error", e.toString());
    }
  }

  public ShootParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // 1. Determine Target based on Alliance
    var alliance = DriverStation.getAlliance();

    // 2. Get Current Robot/Shooter Position
    Pose2d currentPose = drive.getState().Pose;

    InterpolatingTreeMap<Double, Rotation2d> currentShotHoodAngleMap;
    InterpolatingDoubleTreeMap currentShotFlywheelSpeedMap;
    InterpolatingDoubleTreeMap currentTimeOfFlightMap;

    //Determine which target to use
    boolean isRed = alliance.orElse(Alliance.Blue) == Alliance.Red;
    double robotX = currentPose.getX();

    boolean isPastTrench = isRed ? (robotX < 11) : (robotX > 5.5);
    boolean isLeftRight = currentPose.getY() > 4;

    Pose2d targetPose;

    if (isPastTrench) {
      targetPose = isLeftRight ? new Pose2d(1, 7, new Rotation2d()) : new Pose2d(1, 1, new Rotation2d());
      if (isRed) targetPose = FlippingUtil.flipFieldPose(targetPose);
      currentShotHoodAngleMap = passingShotHoodAngleMap;
      currentShotFlywheelSpeedMap = passingShotFlywheelSpeedMap;
      currentTimeOfFlightMap = passingTimeOfFlightMap;
    } else {
      targetPose = isRed ? Constants.fieldPoses.redAllianceHub : Constants.fieldPoses.blueAllianceHub;
      currentShotHoodAngleMap = shotHoodAngleMap;
      currentShotFlywheelSpeedMap = shotFlywheelSpeedMap;
      currentTimeOfFlightMap = timeOfFlightMap;
    }

    // Convert Transform3d to Transform2d safely
    Pose2d shooterPosition = currentPose.transformBy(
        new Transform2d(
            Constants.robotToShooter.getTranslation().toTranslation2d(),
            Constants.robotToShooter.getRotation().toRotation2d()
        )
    );

    double shooterToTargetDistance =
        targetPose.getTranslation().getDistance(shooterPosition.getTranslation()) + Constants.shooterConstants.hubOffset;
    SmartDashboard.putNumber("shooterToTargetDistance", shooterToTargetDistance);

    ChassisSpeeds fieldSpeeds = drive.getState().Speeds;
    double robotRotation = currentPose.getRotation().getRadians();
    Translation2d shooterOffset = Constants.robotToShooter.getTranslation().toTranslation2d();
    double shooterVelocityX = fieldSpeeds.vxMetersPerSecond
        + fieldSpeeds.omegaRadiansPerSecond
            * (shooterOffset.getY() * Math.cos(robotRotation) - shooterOffset.getX() * Math.sin(robotRotation));

    double shooterVelocityY = fieldSpeeds.vyMetersPerSecond
        + fieldSpeeds.omegaRadiansPerSecond
            * (shooterOffset.getX() * Math.cos(robotRotation) - shooterOffset.getY() * Math.sin(robotRotation));
    double initialDistance = targetPose.getTranslation().getDistance(shooterPosition.getTranslation());
    double predictedTOF = currentTimeOfFlightMap.get(initialDistance);

    // Iteratively find the VIRTUAL TARGET
    // We subtract the velocity from the target's position to compensate for ball momentum
    Translation2d virtualTarget = targetPose.getTranslation();

    for (int i = 0; i < 3; i++) { //99.9% convergence
      virtualTarget = targetPose.getTranslation().minus(
          new Translation2d(shooterVelocityX * predictedTOF, shooterVelocityY * predictedTOF)
      );
      double virtualDistance = virtualTarget.getDistance(shooterPosition.getTranslation());
      predictedTOF = currentTimeOfFlightMap.get(virtualDistance);
    }

    // final calculations based on the Virtual Target
    double lookaheadDistance = virtualTarget.getDistance(shooterPosition.getTranslation());

    // robot points at the Virtual Target (this handles the leading/sideways drift)
    Rotation2d targetAngle = virtualTarget.minus(shooterPosition.getTranslation()).getAngle();
    double robotHeadingRadians = targetAngle.getRadians();

    // Interpolate based on the distance to that virtual target
    double hoodAngle = currentShotHoodAngleMap.get(lookaheadDistance).getDegrees();
    double flywheelSpeed = currentShotFlywheelSpeedMap.get(lookaheadDistance);

    //Calculate Target Hood Velocity (Rate of change)
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    lastHoodAngle = hoodAngle;

    latestParameters = new ShootParameters(
        initialDistance,
        hoodAngle,
        flywheelSpeed,
        lookaheadDistance,
        robotHeadingRadians,
        isPastTrench
    );

    return latestParameters;
  }

  /**
   * Resets the cached parameters. Call this at the start of every robot loop.
   */
  public void clearShootingParameters() {
    latestParameters = null;
  }
}