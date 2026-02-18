package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class RunRollers extends Command {

    private final RollerSubsystem rollers;
    private final double speed;

    public RunRollers(RollerSubsystem rollers, double speed) {
        this.rollers = rollers;
        this.speed = speed;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {
        rollers.runPercent(speed);  // percent output
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stop();
    }

    @Override
    public boolean isFinished() {
        return false;  // runs until button released
    }
}
