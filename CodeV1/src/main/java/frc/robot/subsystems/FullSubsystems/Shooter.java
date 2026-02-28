package frc.robot.subsystems.FullSubsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IndexerSubsystem indexer;
  private final HoodSubsystem hood;

  private static final double BASE_SHOOTER_PERCENT = 0.6;
  private static final double KICKER_PERCENT  = 0.9;
  private static final double INDEXER_PERCENT = 0.4;

  // hood increment is 0.2 rotations
  private static final double HOOD_STEP_ROT = 0.2;

  // "every click faster by 0.06"
  // if you REALLY meant 0.6 per click, set this to 0.6
  private static final double SHOOTER_BOOST_PER_STEP = 0.06;

  private static final double READY_FRACTION = 1.0; 
  private static final double READY_TIME_SEC = 0.9;

  private final Debouncer readyDebounce =
      new Debouncer(READY_TIME_SEC, Debouncer.DebounceType.kRising);

  private double expectedRps = 0.0;

  public Shooter(ShooterSubsystem shooter, KickerSubsystem kicker, IndexerSubsystem indexer, HoodSubsystem hood) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
    this.hood = hood;
  }

  private double getShooterPercent() {
    double hoodRot = hood.getTargetRotations(); // 0..5
    double steps = hoodRot / HOOD_STEP_ROT;     // 0..25
    double percent = BASE_SHOOTER_PERCENT + steps * SHOOTER_BOOST_PER_STEP;
    return Math.min(percent, 1.0);              // clamp to 1.0
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/AvgRPS", shooter.getAvgRps());
    Logger.recordOutput("Shooter/ExpectedRPS", expectedRps);
    Logger.recordOutput("Shooter/Ready", isReadyToFeed());
    Logger.recordOutput("Shooter/Percent", getShooterPercent());
    Logger.recordOutput("Shooter/HoodTargetRot", hood.getTargetRotations());
    Logger.recordOutput("Shooter/LeftRPS", shooter.getLeftRps());
    Logger.recordOutput("Shooter/MiddleRPS", shooter.getMiddleRps());
    Logger.recordOutput("Shooter/RightRPS", shooter.getRightRps());
  }

  private void updateExpectedRps() {
    double avg = shooter.getAvgRps();
    if (avg > 5.0) {
      double alpha = 0.02;
      if (expectedRps <= 0.0) {
        expectedRps = avg;
      } else {
        expectedRps = (1.0 - alpha) * expectedRps + alpha * avg;
      }
    }
  }

  public boolean isReadyToFeed() {
    double avg = shooter.getAvgRps();

    if (expectedRps <= 0.0) {
      return readyDebounce.calculate(avg > 40.0); // fallback threshold
    }

    boolean ready = avg >= expectedRps * READY_FRACTION;
    return readyDebounce.calculate(ready);
  }

  public void stopAll() {
    shooter.stop();
    kicker.stop();
    indexer.stop();
    expectedRps = 0.0;
    readyDebounce.calculate(false);
  }

  public Command shootWhileHeld() {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.run(getShooterPercent()), shooter),
        Commands.run(() -> {
          shooter.run(getShooterPercent());
          updateExpectedRps();
        }, shooter).until(this::isReadyToFeed),
        Commands.run(() -> {
          shooter.run(getShooterPercent());
          kicker.run(KICKER_PERCENT);
          indexer.run(INDEXER_PERCENT);
        }, shooter, kicker, indexer)
    ).finallyDo(this::stopAll);
  }
}