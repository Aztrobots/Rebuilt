package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Secuencia completa de disparo: posiciona el wrist, acelera el shooter,
 * activa el feeder cuando el shooter está a velocidad, y opcionalmente
 * auto-apunta al AprilTag más cercano usando ATTRotation.
 *
 * Corre indefinidamente hasta que se suelte el botón (isFinished = false).
 */
public class ShootSequence extends Command {

    private final ShooterSubsystem shooterSub;
    private final IntakeSubsystem intakeSub;
    // RPM objetivo del shooter — TODO: calibrar en robot físico
    private final double targetRPM;
    // Angulo destino del wrist en rotaciones del rotor — TODO: calibrar en robot fisico
    private final double wristAngle;
    // Si true, se activa ATTRotation para alinear el robot con el AprilTag
    private final boolean useAutoAim;
    // Instancia de auto-rotación; se programa/cancela según useAutoAim
    private final ATTRotation attRotation;

    /**
     * @param shooterSub     Subsistema del shooter (ruedas + wrist)
     * @param intakeSub      Subsistema del intake (feeder/indexer)
     * @param targetRPM      RPM objetivo del shooter — TODO: calibrar en robot físico
     * @param wristAngle     Posición del wrist en rotaciones del rotor — TODO: calibrar
     * @param useAutoAim     Si true, activa ATTRotation para alinear con AprilTag
     * @param drivetrain     Drivetrain swerve (solo relevante si useAutoAim=true)
     * @param driveRequest   SwerveRequest field-centric (solo si useAutoAim=true)
     * @param rotationPID    PID de rotación para auto-apuntado (solo si useAutoAim=true)
     * @param maxAngularRate Velocidad angular máxima del drivetrain en rad/s
     * @param limelightName  Nombre de la Limelight para detección de AprilTag
     * @param translationX   Proveedor de velocidad X del piloto (campo-céntrico)
     * @param translationY   Proveedor de velocidad Y del piloto (campo-céntrico)
     */
    public ShootSequence(
        ShooterSubsystem shooterSub,
        IntakeSubsystem intakeSub,
        double targetRPM,
        double wristAngle,
        boolean useAutoAim,
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric driveRequest,
        PIDController rotationPID,
        double maxAngularRate,
        String limelightName,
        Supplier<Double> translationX,
        Supplier<Double> translationY
    ) {
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;
        this.targetRPM = targetRPM;
        this.wristAngle = wristAngle;
        this.useAutoAim = useAutoAim;

        // Crear ATTRotation siempre; solo se programa si useAutoAim=true.
        // El drivetrain es reclamado por ATTRotation cuando se programa, no por ShootSequence.
        this.attRotation = new ATTRotation(
            drivetrain, driveRequest, rotationPID, maxAngularRate,
            limelightName, translationX, translationY
        );

        // ShootSequence reclama shooter e intake; drivetrain lo reclama ATTRotation por separado
        addRequirements(shooterSub, intakeSub);
    }

    @Override
    public void initialize() {
        // Mover el wrist a la posición de disparo antes de acelerar las ruedas
        // TODO: verificar que el ángulo llegue antes de que el shooter esté a velocidad
        shooterSub.setWristPosition(wristAngle);

        // Arrancar el shooter en lazo cerrado a la velocidad objetivo
        shooterSub.setRPM(targetRPM, shooterSub.rsMotor);

        // Activar auto-apuntado al AprilTag si está habilitado
        if (useAutoAim) {
            attRotation.schedule();
        }
    }

    @Override
    public void execute() {
        // Alimentar la nota solo cuando el shooter está a velocidad para evitar disparos lentos
        if (shooterSub.isVelocityWithinTolerance()) {
            intakeSub.setIndexerSpeed(Constants.Shooter.FEEDER_SPEED);
        } else {
            // Mantener feeder detenido mientras el shooter aún acelera
            intakeSub.setIndexerSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Detener el feeder inmediatamente al soltar el botón
        intakeSub.setIndexerSpeed(0);

        // Detener todas las ruedas del shooter (y el wrist queda en posición por brake mode)
        shooterSub.stop();

        // Cancelar el auto-apuntado si estaba activo
        if (useAutoAim) {
            attRotation.cancel();
        }

        // Regresar el wrist a home una vez que el shooter frene por debajo de 100 RPM.
        // Se programa como comando independiente para no bloquear la liberación de recursos.
        // TODO: ajustar el umbral de 100 RPM si el wrist necesita moverse antes o después
        Commands.sequence(
            Commands.waitUntil(
                () -> shooterSub.rsMotor.getVelocity().getValue().in(RPM) < 100.0
            ),
            new WristPos(shooterSub, Constants.Shooter.WRIST_ANGLE_HOME)
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        // El comando corre hasta que se suelte el botón (whileTrue)
        return false;
    }
}
