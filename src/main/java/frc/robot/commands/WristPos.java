package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WristPos extends Command {
    private final ShooterSubsystem sub;
    private final double pos;

    public WristPos(ShooterSubsystem sub, double pos) {
        this.sub = sub;
        this.pos = pos;
        addRequirements(sub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        sub.setWristPosition(pos);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(pos - sub.getWristPosition()) <= 0.15;
    }
}
