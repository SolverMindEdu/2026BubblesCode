package frc.robot.subsystems.FullSubsystems;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.Slapdown;
import frc.robot.subsystems.IndexerSubsystem;
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
  private final IndexerSubsystem indexer;

  private State state = State.UP;
  private State desired = State.UP;
  private static final double ROLLER_INTAKE_RPS = 65.0;
  private static final double INDEXER_ASSIST_PERCENT = 0.2;

  public Intake(IntakeSlapdown slapdown, IntakeRollerSubsystem rollers, IndexerSubsystem indexer) {
    this.slapdown = slapdown;
    this.rollers = rollers;
    this.indexer = indexer;
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

  private void startIntakeMotors() {
    rollers.runIntakeRps(ROLLER_INTAKE_RPS);
    indexer.run(INDEXER_ASSIST_PERCENT);
  }

  private void stopIntakeMotors() {
    rollers.stop();
    indexer.stop();
  }

  public Command goUp() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.UP;
          state = State.MOVING;
        }),
        Commands.runOnce(this::stopIntakeMotors),
        new Slapdown(slapdown, IntakeSlapdown.UP_ROT),
        Commands.runOnce(() -> state = State.UP)
    ).withName("Intake.GoUp");
  }

  public Command goTravel() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.TRAVEL;
          state = State.MOVING;
        }),
        Commands.runOnce(this::stopIntakeMotors),
        new Slapdown(slapdown, IntakeSlapdown.TRAVEL_ROT),
        Commands.runOnce(() -> state = State.TRAVEL)
    ).withName("Intake.GoTravel");
  }

  public Command goDownAndIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.DOWN;
          state = State.MOVING;
        }),
        new Slapdown(slapdown, IntakeSlapdown.DOWN_ROT),
        Commands.runOnce(this::startIntakeMotors),
        Commands.runOnce(() -> state = State.DOWN)
    ).withName("Intake.GoDownAndIntake");
  }

  public Command stopIntakeAndGoTravel() {
    return Commands.sequence(
        Commands.runOnce(this::stopIntakeMotors),
        Commands.runOnce(() -> {
          desired = State.TRAVEL;
          state = State.MOVING;
        }),
        new Slapdown(slapdown, IntakeSlapdown.TRAVEL_ROT),
        Commands.runOnce(() -> state = State.TRAVEL)
    ).withName("Intake.StopIntakeAndGoTravel");
  }

  public Command tapToggleUpTravel() {
    return Commands.defer(
        () -> isUp() ? goTravel() : goUp(),
        Set.of(this)
    ).withName("Intake.TapToggleUpTravel");
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