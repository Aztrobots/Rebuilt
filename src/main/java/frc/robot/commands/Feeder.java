package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Feeder extends Command{
    private final IntakeSubsystem sub;
    
    public Feeder(IntakeSubsystem sub) {
        this.sub = sub;
    }

    @Override
    public void execute() {
        sub.startFeeder();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
