package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IndexerSubsystem indexer;
  private final HoodSubsystem hood;
  private final IntakeSlapdownSubsystem slapdown;
  private final IntakeRollerSubsystem rollers;

  private double targetRps = 0.0;
  private double targetHoodDeg = 0.0;
  private boolean feedingLatched = false;

  public Shooter(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      IndexerSubsystem indexer,
      HoodSubsystem hood,
      IntakeSlapdownSubsystem slapdown,
      IntakeRollerSubsystem rollers
  ) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.hood = hood;
    this.slapdown = slapdown;
    this.rollers = rollers;

    SmartDashboard.putNumber("Demo/ShooterRPS", Constants.Shooter.DEFAULT_TARGET_RPS);
    SmartDashboard.putNumber("Demo/HoodDeg", Constants.Shooter.DEFAULT_HOOD_DEG);
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

  private void updateFixedTargets() {
    targetRps = Math.max(0.0,
        SmartDashboard.getNumber("Demo/ShooterRPS", Constants.Shooter.DEFAULT_TARGET_RPS));
    targetHoodDeg =
        SmartDashboard.getNumber("Demo/HoodDeg", Constants.Shooter.DEFAULT_HOOD_DEG);

    hood.setTargetDegrees(targetHoodDeg);
    shooter.setTargetRps(targetRps);
  }

  public void stopAll() {
    shooter.stop();
    kicker.stop();
    indexer.stop();
    rollers.stop();
    hood.setTargetDegrees(Constants.Hood.MIN_DEG);
    if (slapdown != null) {
      slapdown.travel();
    }
    targetRps = 0.0;
  }

  private Command manageIntakeWhileShooting() {
    if (slapdown == null) {
      return Commands.none();
    }

    final Timer intakeTimer = new Timer();

    return Commands.sequence(
        Commands.runOnce(() -> {
          intakeTimer.restart();
          slapdown.shoot();
          rollers.runIntakeRps(Constants.Shooter.SHOOT_ASSIST_INTAKE_RPS);
        }),
        Commands.run(
            () -> {
              rollers.runIntakeRps(Constants.Shooter.SHOOT_ASSIST_INTAKE_RPS);

              if (intakeTimer.get() < Constants.Shooter.INTAKE_RAISE_DELAY_SEC) {
                slapdown.shoot();
              } else {
                slapdown.upSlow();
              }
            },
            slapdown,
            rollers
        )
    ).finallyDo(() -> {
      intakeTimer.stop();
      slapdown.travel();
      rollers.stop();
    });
  }

  public Command shootWhileHeld() {
    final Timer spinupTimer = new Timer();

    Command mainShoot = Commands.sequence(
        Commands.runOnce(() -> {
          feedingLatched = false;
          spinupTimer.reset();
          spinupTimer.start();
          updateFixedTargets();
        }, hood, shooter),

        Commands.run(() -> {
          updateFixedTargets();

          if (!feedingLatched) {
            if (spinupTimer.hasElapsed(Constants.Shooter.LATCH_MIN_TIME_SEC)
                && shooter.isAtTargetSpeed()) {
              feedingLatched = true;
            }
          }

          if (feedingLatched) {
            kicker.run(Constants.Shooter.KICKER_PERCENT);
            indexer.run(Constants.Shooter.INDEXER_PERCENT);
          } else {
            kicker.stop();
            indexer.stop();
          }
        }, hood, shooter, kicker, indexer)
    );

    return Commands.parallel(
        mainShoot,
        manageIntakeWhileShooting()
    ).finallyDo(() -> {
      spinupTimer.stop();
      feedingLatched = false;
      stopAll();
    });
  }

  public boolean isFeeding() {
    return feedingLatched;
  }

  public boolean isAtSpeed() {
    return shooter.isAtTargetSpeed();
  }
}
