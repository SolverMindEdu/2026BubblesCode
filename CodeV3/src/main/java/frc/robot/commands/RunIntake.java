package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FullSubsystems.Intake;

public class RunIntake extends Command {

  private final Intake intake;

  public RunIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.goDownAndIntake().schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}