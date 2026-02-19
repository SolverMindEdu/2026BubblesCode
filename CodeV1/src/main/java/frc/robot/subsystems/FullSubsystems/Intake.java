package frc.robot.subsystems.FullSubsystems;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Slapdown;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeSlapdown;

public class Intake extends SubsystemBase {

  public enum State {
    UP,
    DOWN,
    MOVING
  }

  private final IntakeSlapdown slapdown;
  private final IntakeRollerSubsystem rollers;

  //States
  private State state = State.UP;
  private State desired = State.UP;

  //Tune 
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

  public boolean isDown() {
    return state == State.DOWN;
  }

  public boolean isUp() {
    return state == State.UP;
  }

  //Toggle button
  public Command toggle() {
    return Commands.defer(() -> {
      // Flip desired every press
      desired = (desired == State.UP) ? State.DOWN : State.UP;

      if (desired == State.DOWN) {
        // UP -> DOWN
        return Commands.sequence(
            Commands.runOnce(() -> state = State.MOVING),

            // Move down (if pressed again this command will be canceled and reversed)
            new Slapdown(slapdown, IntakeSlapdown.DOWN_ROT),

            // Once down start rollers
            Commands.runOnce(() -> rollers.runIntake(ROLLER_INTAKE_PERCENT), rollers),

            Commands.runOnce(() -> state = State.DOWN)
        );
      } else {
        // DOWN -> UP
        return Commands.sequence(
            Commands.runOnce(() -> state = State.MOVING),

            // Stop rollers
            Commands.runOnce(rollers::stop, rollers),

            // Move up
            new Slapdown(slapdown, IntakeSlapdown.UP_ROT),

            Commands.runOnce(() -> state = State.UP)
        );
      }
    }, Set.of(slapdown, rollers));
  }

  //Makes sure only when down start moving
  public Command goDown() {
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

  //Make sure stop rollers when up
  public Command goUp() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          desired = State.UP;
          state = State.MOVING;
        }),
        Commands.runOnce(rollers::stop, rollers),
        new Slapdown(slapdown, IntakeSlapdown.UP_ROT),
        Commands.runOnce(() -> state = State.UP)
    );
  }
}