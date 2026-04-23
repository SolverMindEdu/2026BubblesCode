package frc.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

public class ShotCalculator extends SubsystemBase {
  private static ShotCalculator instance;
  private final CommandSwerveDrivetrain drive;

  private double lastHoodAngle = Double.NaN;
  private ShootParameters latestParameters = null;

  public ShotCalculator(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  public static ShotCalculator getInstance(CommandSwerveDrivetrain drive) {
    if (instance == null) {
      instance = new ShotCalculator(drive);
    }
    return instance;
  }

  public static ShotCalculator getInstance() {
    return instance;
  }

  public record ShootParameters(
      double initialDistance,
      double hoodAngle,
      double flywheelSpeed,
      double lookaheadDistance,
      double robotHeadingRadians,
      boolean isPassing
  ) {}

  private static final InterpolatingTreeMap<Double, Rotation2d> passingShotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingShotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Add real passing values here when ready
    passingShotHoodAngleMap.put(7.5, Rotation2d.fromDegrees(160.0));
    passingShotHoodAngleMap.put(8.5, Rotation2d.fromDegrees(230.0));
    passingShotHoodAngleMap.put(9.5, Rotation2d.fromDegrees(320.0));
    passingShotFlywheelSpeedMap.put(7.5, 70.0);
    passingShotFlywheelSpeedMap.put(8.5, 73.0);
    passingShotFlywheelSpeedMap.put(9.0, 80.0);
    passingTimeOfFlightMap.put(1.64227, 0.93);
    passingTimeOfFlightMap.put(7.5, 1.10);
    passingTimeOfFlightMap.put(8.5, 1.30);
    passingTimeOfFlightMap.put(9.5, 1.30);
  }

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    shotHoodAngleMap.put(1.0, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.0, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.2, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.5, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.8, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(3.0, Rotation2d.fromDegrees(60.0));
    shotHoodAngleMap.put(3.2, Rotation2d.fromDegrees(80.0));
    shotHoodAngleMap.put(3.5, Rotation2d.fromDegrees(100.0));
    shotHoodAngleMap.put(3.8, Rotation2d.fromDegrees(130.0));
    shotHoodAngleMap.put(4.0, Rotation2d.fromDegrees(140.0));
    shotHoodAngleMap.put(4.2, Rotation2d.fromDegrees(160.0));
    shotHoodAngleMap.put(4.4, Rotation2d.fromDegrees(190.0));
    shotHoodAngleMap.put(4.6, Rotation2d.fromDegrees(210.0));
    shotHoodAngleMap.put(5.2, Rotation2d.fromDegrees(250.0));

    shotFlywheelSpeedMap.put(5.2, 64.0);
    shotFlywheelSpeedMap.put(4.4, 62.0);
    shotFlywheelSpeedMap.put(4.2, 58.0);
    shotFlywheelSpeedMap.put(3.9, 59.5);
    shotFlywheelSpeedMap.put(3.5, 56.0);
    shotFlywheelSpeedMap.put(3.2, 55.5);
    shotFlywheelSpeedMap.put(3.0, 55.0);
    shotFlywheelSpeedMap.put(2.8, 54.0);
    shotFlywheelSpeedMap.put(2.5, 53.5);
    shotFlywheelSpeedMap.put(2.2, 52.0);
    shotFlywheelSpeedMap.put(2.0, 48.5);

    timeOfFlightMap.put(1.0, 0.28);
  }

  @Override
  public void periodic() {
    clearShootingParameters();

    try {
      ShootParameters p = getParameters();
      SmartDashboard.putNumber("Shot/InitialDistance", p.initialDistance());
      SmartDashboard.putNumber("Shot/LookaheadDistance", p.lookaheadDistance());
      SmartDashboard.putNumber("Shot/HoodAngleDeg", p.hoodAngle());
      SmartDashboard.putNumber("Shot/FlywheelRPS", p.flywheelSpeed());
      SmartDashboard.putNumber("Shot/HeadingDeg", Units.radiansToDegrees(p.robotHeadingRadians()));
      SmartDashboard.putBoolean("Shot/IsPassing", p.isPassing());
      SmartDashboard.putString("Shot/Error", "");
    } catch (Exception e) {
      SmartDashboard.putString("Shot/Error", e.toString());
    }
  }

  public ShootParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d currentPose = getSafePose();
    ChassisSpeeds fieldSpeeds = getSafeSpeeds();

    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.orElse(Alliance.Blue) == Alliance.Red;

    double robotX = currentPose.getX();
    boolean isPastTrench = isRed ? (robotX < 11.0) : (robotX > 5.5);
    boolean isLeftRight = currentPose.getY() > 4.0;

    Pose2d passingTargetPose = isLeftRight
        ? new Pose2d(1.0, 7.0, new Rotation2d())
        : new Pose2d(1.0, 1.0, new Rotation2d());

    if (isRed) {
      passingTargetPose = FlippingUtil.flipFieldPose(passingTargetPose);
    }

    Pose2d normalTargetPose = isRed
        ? Constants.fieldPoses.redAllianceHub
        : Constants.fieldPoses.blueAllianceHub;

    Pose2d shooterPosition = currentPose.transformBy(
        new Transform2d(
            Constants.robotToShooter.getTranslation().toTranslation2d(),
            Constants.robotToShooter.getRotation().toRotation2d()
        )
    );

    double robotRotation = currentPose.getRotation().getRadians();
    Translation2d shooterOffset = Constants.robotToShooter.getTranslation().toTranslation2d();

    double shooterVelocityX =
        fieldSpeeds.vxMetersPerSecond
            + fieldSpeeds.omegaRadiansPerSecond
                * (shooterOffset.getY() * Math.cos(robotRotation)
                    - shooterOffset.getX() * Math.sin(robotRotation));

    double shooterVelocityY =
        fieldSpeeds.vyMetersPerSecond
            + fieldSpeeds.omegaRadiansPerSecond
                * (shooterOffset.getX() * Math.cos(robotRotation)
                    - shooterOffset.getY() * Math.sin(robotRotation));

    ShootParameters passingParams = tryCalculateShot(
        passingTargetPose,
        shooterPosition,
        shooterVelocityX,
        shooterVelocityY,
        passingShotHoodAngleMap,
        passingShotFlywheelSpeedMap,
        passingTimeOfFlightMap,
        true
    );

    ShootParameters normalParams = tryCalculateShot(
        normalTargetPose,
        shooterPosition,
        shooterVelocityX,
        shooterVelocityY,
        shotHoodAngleMap,
        shotFlywheelSpeedMap,
        timeOfFlightMap,
        false
    );

    if (isPastTrench && passingParams != null) {
      latestParameters = passingParams;
      return latestParameters;
    }

    if (normalParams != null) {
      latestParameters = normalParams;
      return latestParameters;
    }

    if (passingParams != null) {
      latestParameters = passingParams;
      return latestParameters;
    }

    latestParameters = defaultParameters(currentPose);
    return latestParameters;
  }

  private ShootParameters tryCalculateShot(
      Pose2d targetPose,
      Pose2d shooterPosition,
      double shooterVelocityX,
      double shooterVelocityY,
      InterpolatingTreeMap<Double, Rotation2d> hoodMap,
      InterpolatingDoubleTreeMap flywheelMap,
      InterpolatingDoubleTreeMap tofMap,
      boolean isPassing
  ) {
    try {
      double initialDistance =
          targetPose.getTranslation().getDistance(shooterPosition.getTranslation());

      Double predictedTOFObj = getMapValue(tofMap, initialDistance);
      if (predictedTOFObj == null || !Double.isFinite(predictedTOFObj)) {
        return null;
      }

      double predictedTOF = predictedTOFObj;
      Translation2d virtualTarget = targetPose.getTranslation();

      for (int i = 0; i < 3; i++) {
        virtualTarget = targetPose.getTranslation().minus(
            new Translation2d(shooterVelocityX * predictedTOF, shooterVelocityY * predictedTOF)
        );

        double virtualDistance = virtualTarget.getDistance(shooterPosition.getTranslation());
        Double nextTOF = getMapValue(tofMap, virtualDistance);

        if (nextTOF == null || !Double.isFinite(nextTOF)) {
          return null;
        }

        predictedTOF = nextTOF;
      }

      double lookaheadDistance = virtualTarget.getDistance(shooterPosition.getTranslation());
      Rotation2d targetAngle = virtualTarget.minus(shooterPosition.getTranslation()).getAngle();
      double robotHeadingRadians = targetAngle.getRadians() + Math.PI + Units.degreesToRadians(6);

      Rotation2d hoodRotation = getMapValue(hoodMap, lookaheadDistance);
      Double flywheelSpeedObj = getMapValue(flywheelMap, lookaheadDistance);

      if (hoodRotation == null || flywheelSpeedObj == null || !Double.isFinite(flywheelSpeedObj)) {
        return null;
      }

      double hoodAngle = hoodRotation.getDegrees();
      if (hoodAngle < 0.0) {
        hoodAngle += 360.0;
      }
      double flywheelSpeed = flywheelSpeedObj;

      if (Double.isNaN(lastHoodAngle)) {
        lastHoodAngle = hoodAngle;
      }
      lastHoodAngle = hoodAngle;

      return new ShootParameters(
          initialDistance,
          hoodAngle,
          flywheelSpeed,
          lookaheadDistance,
          robotHeadingRadians,
          isPassing
      );
    } catch (Exception e) {
      return null;
    }
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  private Pose2d getSafePose() {
    try {
      if (drive == null || drive.getState() == null || drive.getState().Pose == null) {
        return new Pose2d();
      }
      return drive.getState().Pose;
    } catch (Exception e) {
      return new Pose2d();
    }
  }

  private ChassisSpeeds getSafeSpeeds() {
    try {
      if (drive == null || drive.getState() == null || drive.getState().Speeds == null) {
        return new ChassisSpeeds();
      }
      return drive.getState().Speeds;
    } catch (Exception e) {
      return new ChassisSpeeds();
    }
  }

  private Rotation2d getMapValue(InterpolatingTreeMap<Double, Rotation2d> map, double key) {
    try {
      return map.get(key);
    } catch (Exception e) {
      return null;
    }
  }

  private Double getMapValue(InterpolatingDoubleTreeMap map, double key) {
    try {
      return map.get(key);
    } catch (Exception e) {
      return null;
    }
  }

  private ShootParameters defaultParameters(Pose2d currentPose) {
    return new ShootParameters(
        0.0,
        Constants.Shooter.DEFAULT_HOOD_DEG,
        Constants.Shooter.DEFAULT_TARGET_RPS,
        0.0,
        currentPose.getRotation().getRadians(),
        false
    );
  }
}