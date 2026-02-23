package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    private final ShooterSubsystem sub;
    private final double rpm;

    public StartShooter(ShooterSubsystem sub, double rpm) {
        this.sub = sub;
        this.rpm = rpm;
        addRequirements(sub);
    }

    @Override
    public void execute() {
        sub.setRPM(rpm, sub.rsMotor);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
