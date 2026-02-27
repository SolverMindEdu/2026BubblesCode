package frc.robot.subsystems.FullSubsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter extends SubsystemBase {

  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IndexerSubsystem indexer;

  private static final double SHOOTER_PERCENT = 0.8;
  private static final double KICKER_PERCENT  = 0.9;
  private static final double INDEXER_PERCENT = 0.1;

  private static final double READY_FRACTION = 0.90; 
  private static final double READY_TIME_SEC = 0.10;

  private final Debouncer readyDebounce =
      new Debouncer(READY_TIME_SEC, Debouncer.DebounceType.kRising);

  private double expectedRps = 0.0;

  public Shooter(ShooterSubsystem shooter, KickerSubsystem kicker, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.indexer = indexer;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/AvgRPS", shooter.getAvgRps());
    Logger.recordOutput("Shooter/ExpectedRPS", expectedRps);
    Logger.recordOutput("Shooter/Ready", isReadyToFeed());
    Logger.recordOutput("Shooter/Percent", SHOOTER_PERCENT);
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
      return readyDebounce.calculate(avg > 40.0); //fallback threshold
    }

    boolean ready = avg >= expectedRps * READY_FRACTION;
    return readyDebounce.calculate(ready);
  }

  public void stopAll() {
    shooter.stop();
    kicker.stop();
    indexer.stop();
    expectedRps = 0.0; //reset each time you spin up
    readyDebounce.calculate(false);
  }

  public Command shootWhileHeld() {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.run(SHOOTER_PERCENT), shooter),
        Commands.run(() -> {
          shooter.run(SHOOTER_PERCENT);
          updateExpectedRps();
        }, shooter).until(this::isReadyToFeed),
        Commands.run(() -> {
          shooter.run(SHOOTER_PERCENT);
          kicker.run(KICKER_PERCENT);
          indexer.run(INDEXER_PERCENT);
        }, shooter, kicker, indexer)
    ).finallyDo(this::stopAll);
  }
}