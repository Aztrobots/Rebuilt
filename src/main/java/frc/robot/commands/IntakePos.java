package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePos extends Command {
    private final IntakeSubsystem sub;
    private final double pos;

    public IntakePos(IntakeSubsystem sub, double pos) {
        this.sub = sub;
        this.pos = pos;
    }

    @Override
    public void execute() {
        sub.setPosition(pos);
    }

    @Override
    public void end(boolean isFinished) {}

    @Override
    public boolean isFinished() {
        return Math.abs(pos - sub.getPosition()) <= 0.15;
    }
}
