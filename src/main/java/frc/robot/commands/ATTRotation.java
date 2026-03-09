package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ATTRotation extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController limelightPID;
    private final double maxAngularRate;
    private final String limelightName;
    // Proveedores de traducción del piloto (eje X e Y del joystick izquierdo)
    private final Supplier<Double> translationX;
    private final Supplier<Double> translationY;

    public ATTRotation(CommandSwerveDrivetrain drivetrain,
                       SwerveRequest.FieldCentric driveRequest,
                       PIDController limelightPID,
                       double maxAngularRate,
                       String limelightName,
                       Supplier<Double> translationX,
                       Supplier<Double> translationY) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.limelightPID = limelightPID;
        this.maxAngularRate = maxAngularRate;
        this.limelightName = limelightName;
        this.translationX = translationX;
        this.translationY = translationY;
        // Declarar el requerimiento del drivetrain para evitar conflictos con otros comandos
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Configurar pipeline 1 (AprilTags) al inicio del comando
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX(limelightName);
        double rotationOutput = 0.0;

        if (LimelightHelpers.getTV(limelightName)) {
            // Calcular la corrección rotacional con PID usando el error horizontal de la Limelight
            rotationOutput = limelightPID.calculate(tx, Constants.Limelights.AprilTagLimits.XError);
            // Limitar la salida rotacional al máximo permitido
            rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);

            System.out.println("tx: " + tx + " | rotOutput: " + rotationOutput);
        }
        // Si no hay AprilTag visible, se pasa rotationOutput = 0.0 y se preserva la traslación del piloto

        // Preservar la traslación del piloto; solo controlar la rotación con el PID
        drivetrain.setControl(
            driveRequest
                .withVelocityX(translationX.get())
                .withVelocityY(translationY.get())
                .withRotationalRate(rotationOutput)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Al terminar, detener completamente el drivetrain
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        // El comando corre indefinidamente hasta que se suelte el botón
        return false;
    }
}
