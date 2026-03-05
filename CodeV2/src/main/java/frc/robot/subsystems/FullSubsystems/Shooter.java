package frc.robot.subsystems.FullSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.ShotCalculator.ShootParameters;
import frc.robot.subsystems.IntakeSlapdown;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IndexerSubsystem indexer;
  private final HoodSubsystem hood;
  private final ShotCalculator shotCalc;
  private final IntakeSlapdown slapdown;

  private static final double KICKER_PERCENT  = 0.9;
  private static final double INDEXER_PERCENT = 0.8;

  private static final double DEFAULT_TARGET_RPS = 60.0;
  private static final double DEFAULT_HOOD_DEG = 10.0;

  private static final double PULSE_PERIOD_SEC = 0.5;
  private static final double PULSE_HALF_SEC = PULSE_PERIOD_SEC / 2.0;

  private double targetRps = 0.0;
  private double targetHoodDeg = 0.0;

  public Shooter(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      IndexerSubsystem indexer,
      HoodSubsystem hood,
      ShotCalculator shotCalc,
      IntakeSlapdown slapdown
  ) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.hood = hood;
    this.shotCalc = shotCalc;
    this.slapdown = slapdown;
  }

  public void setTargetRps(double rps) {
    targetRps = Math.max(0.0, rps);
    shooter.setTargetRps(targetRps);
  }

  public double getTargetRps() {
    return targetRps;
  }

  public double getTargetHoodDeg() {
    return targetHoodDeg;
  }

  private void updateTargetsFromShotCalc() {
    if (shotCalc == null) {
      targetRps = DEFAULT_TARGET_RPS;
      targetHoodDeg = DEFAULT_HOOD_DEG;
      hood.setTargetDegrees(targetHoodDeg);
      shooter.setTargetRps(targetRps);
      return;
    }

    ShootParameters p = shotCalc.getParameters();

    targetRps = Math.max(0.0, p.flywheelSpeed());
    targetHoodDeg = p.hoodAngle();              

    hood.setTargetDegrees(targetHoodDeg);
    shooter.setTargetRps(targetRps);

    Logger.recordOutput("Shot/InitialDistance", p.initialDistance());
    Logger.recordOutput("Shot/LookaheadDistance", p.lookaheadDistance());
    Logger.recordOutput("Shot/HoodAngleDeg", p.hoodAngle());
    Logger.recordOutput("Shot/FlywheelRPS", p.flywheelSpeed());
    Logger.recordOutput("Shot/HeadingRad", p.robotHeadingRadians());
    Logger.recordOutput("Shot/IsPassing", p.isPassing());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/AvgRPS", shooter.getAvgRps());
    Logger.recordOutput("Shooter/TargetRPS", targetRps);
    Logger.recordOutput("Shooter/AtSpeed", shooter.isAtTargetSpeed());
    Logger.recordOutput("Shooter/HoodTargetDeg", hood.getTargetDegrees());
    Logger.recordOutput("Shooter/LeftRPS", shooter.getLeftRps());
    Logger.recordOutput("Shooter/MiddleRPS", shooter.getMiddleRps());
    Logger.recordOutput("Shooter/RightRPS", shooter.getRightRps());
  }

  public void stopAll() {
    shooter.stop();
    kicker.stop();
    indexer.stop();
    if (slapdown != null) {
      slapdown.travel();
    }
    targetRps = 0.0;
  }
  private Command pulseIntakeWhileShooting() {
    if (slapdown == null) return Commands.none();

    return Commands.sequence(
        Commands.runOnce(slapdown::shoot, slapdown),
        Commands.waitSeconds(PULSE_HALF_SEC),
        Commands.runOnce(slapdown::travel, slapdown),
        Commands.waitSeconds(PULSE_HALF_SEC)
    ).repeatedly()
     .finallyDo(() -> slapdown.travel());
  }
  public Command shootWhileHeld() {
    Command mainShoot = Commands.sequence(
        // Prime targets once at start
        Commands.runOnce(() -> {
          try {
            updateTargetsFromShotCalc();
          } catch (Exception e) {
            Logger.recordOutput("Shot/Error", e.toString());
            targetRps = DEFAULT_TARGET_RPS;
            targetHoodDeg = DEFAULT_HOOD_DEG;
            hood.setTargetDegrees(targetHoodDeg);
            shooter.setTargetRps(targetRps);
          }
        }, hood, shooter),
        Commands.run(() -> {
          try {
            updateTargetsFromShotCalc();
          } catch (Exception e) {
            Logger.recordOutput("Shot/Error", e.toString());
            if (targetRps <= 0.0) {
              targetRps = DEFAULT_TARGET_RPS;
              shooter.setTargetRps(targetRps);
            }
            if (targetHoodDeg <= 0.0) {
              targetHoodDeg = DEFAULT_HOOD_DEG;
              hood.setTargetDegrees(targetHoodDeg);
            }
          }
        }, hood, shooter).until(shooter::isAtTargetSpeed),

        Commands.run(() -> {
          try {
            updateTargetsFromShotCalc();
          } catch (Exception e) {
            Logger.recordOutput("Shot/Error", e.toString());
          }
          kicker.run(KICKER_PERCENT);
          indexer.run(INDEXER_PERCENT);
        }, hood, shooter, kicker, indexer)
    );

    return Commands.parallel(
        mainShoot,
        pulseIntakeWhileShooting()
    ).finallyDo(this::stopAll);
  }
}