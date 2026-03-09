package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    // Solo se usa el subsistema del shooter pasado por el constructor
    private final ShooterSubsystem sub;
    private final double speed;
    private boolean isCalculationEnabled;
    private static double time = 0.0;

    public StartShooter(ShooterSubsystem sub, double speed, boolean isCalculationEnabled) {
        this.sub = sub;
        this.speed = speed;
        this.isCalculationEnabled = isCalculationEnabled;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        String limelightName = Constants.Limelights.shooterLimelightName;

        // TODO: se necesita recibir un IndexerSubsystem como parámetro en el constructor para poder llamar setIndexerSpeed(0.75)
        // intakeSub.setIndexerSpeed(0.75);

        if (time >= 0.5) {
            if (isCalculationEnabled) {
                // TODO: se necesita integración de visión; se requiere pasar RobotContainer o AprilTagTracker al constructor
                // Pose3d targetLocation = m_RobotContainer.getClosestAprilTag().getTargetPose_CameraSpace();
                // double tz = targetLocation.getZ();
                // if (LimelightHelpers.getTV(limelightName)) sub.setRPM(
                //     sub.calculateRPMWithITM(limelightName, tz),
                //     sub.rsMotor
                // );

                // Por ahora se usa velocidad fija como respaldo hasta integrar la visión
                sub.set(speed);
            }

            else sub.set(speed);
        }
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
