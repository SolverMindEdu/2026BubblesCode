package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSlapdown;

public class Slapdown extends Command {
  private final IntakeSlapdown slapdown;
  private final double targetRotations;
  private final double timeoutSec;

  private final Timer timer = new Timer();

  public Slapdown(IntakeSlapdown slapdown, double targetRotations) {
    this(slapdown, targetRotations, 1); // timeout value
  }

  public Slapdown(IntakeSlapdown slapdown, double targetRotations, double timeoutSec) {
    this.slapdown = slapdown;
    this.targetRotations = MathUtil.clamp(
        targetRotations,
        IntakeSlapdown.UP_LIMIT_ROT,
        IntakeSlapdown.DOWN_LIMIT_ROT
    );
    this.timeoutSec = timeoutSec;
    addRequirements(slapdown);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    slapdown.goToRotations(targetRotations);
  }

  @Override
  public boolean isFinished() {
    return slapdown.atSetpoint(targetRotations) || timer.hasElapsed(timeoutSec);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // holding position
    // slapdown.stop(); // only if you explicitly want to stop moving it
  }
}
