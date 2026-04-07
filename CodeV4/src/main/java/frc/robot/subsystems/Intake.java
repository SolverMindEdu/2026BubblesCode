package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public enum State {
    UP,
    TRAVEL,
    DOWN,
    MOVING,
    REVERSE
  }

  private final IntakeSlapdownSubsystem slapdown;
  private final IntakeRollerSubsystem rollers;
  private final IndexerSubsystem indexer;
  private final LEDs leds;

  private State state = State.UP;
  private State desired = State.UP;

  public Intake(
      IntakeSlapdownSubsystem slapdown,
      IntakeRollerSubsystem rollers,
      IndexerSubsystem indexer,
      LEDs leds
  ) {
    this.slapdown = slapdown;
    this.rollers = rollers;
    this.indexer = indexer;
    this.leds = leds;
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

  private void startIntake() {
    rollers.runIntakeRps(Constants.Intake.ROLLER_RPS);
    indexer.run(Constants.Intake.INDEXER_PERCENT);
    leds.setState(LEDs.LEDState.INTAKING_GREEN_FLASH);
  }

  private void stopIntake() {
    rollers.stop();
    indexer.stop();
    leds.setState(LEDs.LEDState.IDLE_BLUE);
  }

  private void startReverse() {
    rollers.runIntakeRps(-Constants.Intake.ROLLER_RPS);
    indexer.run(-Constants.Intake.INDEXER_PERCENT);
    leds.setState(LEDs.LEDState.REVERSE_RED_FLASH);
  }

  public Command goUp() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.UP;
          state = State.MOVING;
          leds.setState(LEDs.LEDState.IDLE_BLUE);
        }),
        Commands.runOnce(this::stopIntake),
        Commands.runOnce(() -> slapdown.goToRotations(Constants.Intake.UP_ROT)),
        Commands.waitUntil(() -> slapdown.atSetpoint(Constants.Intake.UP_ROT)),
        Commands.runOnce(() -> state = State.UP)
    ).withName("Intake.GoUp");
  }

  public Command goTravel() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.TRAVEL;
          state = State.MOVING;
          leds.setState(LEDs.LEDState.IDLE_BLUE);
        }),
        Commands.runOnce(this::stopIntake),
        Commands.runOnce(() -> slapdown.goToRotations(Constants.Intake.TRAVEL_ROT)),
        Commands.waitUntil(() -> slapdown.atSetpoint(Constants.Intake.TRAVEL_ROT)),
        Commands.runOnce(() -> state = State.TRAVEL)
    ).withName("Intake.GoTravel");
  }

  public Command goDownAndIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.DOWN;
          state = State.MOVING;
          leds.setState(LEDs.LEDState.IDLE_BLUE);
        }),
        Commands.runOnce(this::stopIntake),
        Commands.runOnce(() -> slapdown.goToRotations(Constants.Intake.DOWN_ROT)),
        Commands.waitUntil(() -> slapdown.atSetpoint(Constants.Intake.DOWN_ROT)),
        Commands.runOnce(() -> {
          state = State.DOWN;
          startIntake();
        })
    ).withName("Intake.GoDownAndIntake");
  }

  public Command holdDownAndIntake() {
    return Commands.run(
        () -> {
          desired = State.DOWN;
          slapdown.goToRotations(Constants.Intake.DOWN_ROT);

          if (slapdown.isDown()) {
            state = State.DOWN;
            startIntake();
          } else {
            state = State.MOVING;
            stopIntake();
          }
        },
        this
    ).withName("Intake.HoldDownAndIntake");
  }

  public Command holdReverseJamClear() {
    return Commands.run(
        () -> {
          desired = State.REVERSE;
          slapdown.goToRotations(Constants.Intake.DOWN_ROT);

          if (slapdown.isDown()) {
            state = State.REVERSE;
            startReverse();
          } else {
            state = State.MOVING;
            stopIntake();
          }
        },
        this
    ).withName("Intake.HoldReverseJamClear");
  }

  public Command stopIntakeAndGoTravel() {
    return Commands.sequence(
        Commands.runOnce(this::stopIntake),
        goTravel()
    ).withName("Intake.StopIntakeAndGoTravel");
  }

  public Command tapToggleUpTravel() {
    return Commands.defer(
        () -> isUp() ? goTravel() : goUp(),
        Set.of(this)
    ).withName("Intake.TapToggleUpTravel");
  }
}