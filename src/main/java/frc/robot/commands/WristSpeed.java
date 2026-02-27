package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WristSpeed extends Command {
    private final ShooterSubsystem sub;
    private final double speed;

    public WristSpeed(ShooterSubsystem sub, double speed) {
        this.sub = sub;
        this.speed = speed;
        addRequirements(sub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        sub.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sub.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
