package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Feeder extends Command{
    private final double speed;
    private final IntakeSubsystem sub;
    
    public Feeder(IntakeSubsystem sub, double speed) {
        this.speed = speed;
        this.sub = sub;
    }

    @Override
    public void execute() {
        sub.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
