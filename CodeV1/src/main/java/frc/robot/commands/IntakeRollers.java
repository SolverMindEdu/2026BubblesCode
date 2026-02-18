package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class IntakeRollers extends Command {

    private final IntakeRollerSubsystem rollers;
    private final double speed;

    public IntakeRollers(IntakeRollerSubsystem rollers, double speed) {
        this.rollers = rollers;
        this.speed = speed;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {
        rollers.runIntake(speed); 
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stop();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
