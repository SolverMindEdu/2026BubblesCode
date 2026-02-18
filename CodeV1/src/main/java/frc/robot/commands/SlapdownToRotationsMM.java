package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSlapdown;

public class SlapdownToRotationsMM extends Command {
  private final IntakeSlapdown slapdown;
  private final double targetRotations;

  public SlapdownToRotationsMM(IntakeSlapdown slapdown, double targetRotations) {
    this.slapdown = slapdown;
    this.targetRotations = targetRotations;
    addRequirements(slapdown);
  }

  @Override
  public void initialize() {
    slapdown.goToRotations(targetRotations);
  }

  @Override
  public boolean isFinished() {
    return slapdown.atSetpoint(targetRotations);
  }

  @Override
  public void end(boolean interrupted) {
    // Keep holding at target (Motion Magic maintains closed-loop position).
    // If you want it to fully stop output instead, uncomment:
    // slapdown.stop();
  }
}
