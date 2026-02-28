package frc.robot.subsystems.FullSubsystems;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Slapdown;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdown;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum State {
    UP,
    TRAVEL,
    DOWN,
    MOVING
  }

  private final IntakeSlapdown slapdown;
  private final IntakeRollerSubsystem rollers;

  private State state = State.UP;
  private State desired = State.UP;

  private static final double ROLLER_INTAKE_PERCENT = 0.8;

  public Intake(IntakeSlapdown slapdown, IntakeRollerSubsystem rollers) {
    this.slapdown = slapdown;
    this.rollers = rollers;
  }

  public State getState() {
    return state;
  }

  public State getDesired() {
    return desired;
  }

  public boolean isUp() {
    return state == State.UP;
  }

  public boolean isTravel() {
    return state == State.TRAVEL;
  }

  public boolean isDown() {
    return state == State.DOWN;
  }

  public Command goUp() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.UP;
          state = State.MOVING;
        }),
        Commands.runOnce(rollers::stop, rollers), // rollers never run when going up
        new Slapdown(slapdown, IntakeSlapdown.UP_ROT),
        Commands.runOnce(() -> state = State.UP)
    );
  }

  public Command goTravel() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.TRAVEL;
          state = State.MOVING;
        }),
        Commands.runOnce(rollers::stop, rollers), // rollers off in travel
        new Slapdown(slapdown, IntakeSlapdown.TRAVEL_ROT),
        Commands.runOnce(() -> state = State.TRAVEL)
    );
  }

  public Command goDownAndIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.DOWN;
          state = State.MOVING;
        }),
        new Slapdown(slapdown, IntakeSlapdown.DOWN_ROT),
        Commands.runOnce(() -> rollers.runIntake(ROLLER_INTAKE_PERCENT), rollers),
        Commands.runOnce(() -> state = State.DOWN)
    );
  }

  public Command stopIntakeAndGoTravel() {
    return Commands.sequence(
        Commands.runOnce(rollers::stop, rollers),
        Commands.runOnce(() -> {
          desired = State.TRAVEL;
          state = State.MOVING;
        }),
        new Slapdown(slapdown, IntakeSlapdown.TRAVEL_ROT),
        Commands.runOnce(() -> state = State.TRAVEL)
    );
  }

  public Command tapToggleUpTravel() {
    return Commands.defer(() -> {
      if (isUp()) {
        return goTravel();
      } else {
        // from travel tap makes go up
        return goUp();
      }
    }, Set.of(this, slapdown, rollers));
  }

  @Override
  public void periodic() {
      Logger.recordOutput("Intake/State", state.toString());
      Logger.recordOutput("Intake/DesiredState", desired.toString());
      Logger.recordOutput("Intake/SlapdownPositionRot", slapdown.getPositionRotations());
      Logger.recordOutput("Intake/IsUp", isUp());
      Logger.recordOutput("Intake/IsTravel", isTravel());
      Logger.recordOutput("Intake/IsDown", isDown());
  }
}