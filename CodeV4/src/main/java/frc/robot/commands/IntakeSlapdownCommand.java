package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSlapdownSubsystem;

public class IntakeSlapdownCommand extends Command {

  private final IntakeSlapdownSubsystem slapdown;
  private final double targetRotations;

  public IntakeSlapdownCommand(IntakeSlapdownSubsystem slapdown, double targetRotations) {
    this.slapdown = slapdown;
    this.targetRotations = MathUtil.clamp(
        targetRotations,
        Constants.Intake.UP_LIMIT_ROT,
        Constants.Intake.DOWN_LIMIT_ROT
    );

    addRequirements(slapdown);
  }

  @Override
  public void execute() {
    slapdown.goToRotations(targetRotations);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    slapdown.stop();
  }
}