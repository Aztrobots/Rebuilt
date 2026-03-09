package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 *
 * Flujo:
 *   Odometría (encoders + giroscopio) → estimación base
 *   Limelight (AprilTags)             → corrección periódica
 *   Kalman Filter (dentro de CTRE)    → pose final fusionada
 */
public class VisionSubsystem extends SubsystemBase {

    // Drivetrain al que se le inyectan las mediciones de visión
    private final CommandSwerveDrivetrain drivetrain;

    // Nombre de la cámara Limelight configurada en la interfaz web
    private final String limelightName;

    // Umbral máximo de distancia aceptable para una medición (en metros).
    // Más lejos = más ruido en la estimación de pose → rechazamos.
    private static final double MAX_TAG_DIST_METERS = 4.0;

    // Umbral de distancia para elegir entre std devs "confianza alta" o "baja"
    private static final double NEAR_TAG_DIST_METERS = 2.0;

    // Desviaciones estándar para tags cercanos (≤ 2 m).
    // Valores pequeños = confiamos más en la cámara que en la odometría.
    // Orden: [x (m), y (m), theta (rad)]
    private static final double NEAR_STD_X     = 0.3;
    private static final double NEAR_STD_Y     = 0.3;
    private static final double NEAR_STD_THETA = 0.5;

    // Desviaciones estándar para tags lejanos (> 2 m).
    // Valores grandes = confiamos más en la odometría que en la cámara.
    private static final double FAR_STD_X     = 1.0;
    private static final double FAR_STD_Y     = 1.0;
    private static final double FAR_STD_THETA = 2.0;

    /**
     * Crea el subsistema de visión.
     *
     * @param drivetrain   El swerve drivetrain al que se añadirán las mediciones.
     * @param limelightName Nombre de la Limelight (ej. "limelight", "limelight2").
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
    }

    /**
     * Se ejecuta cada ciclo del robot (~20 ms).
     *
     * Lee la estimación de pose desde la Limelight, aplica filtros de calidad,
     * y si la medición es confiable la inyecta al filtro de Kalman del drivetrain.
     */
    @Override
    public void periodic() {
        // Obtenemos la alianza actual para usar el sistema de coordenadas correcto.
        // WPILib define el origen del campo en la esquina de la alianza azul,
        // por lo que usamos botpose_wpiblue para azul y botpose_wpired para rojo.
        Optional<Alliance> alliance = DriverStation.getAlliance();

        PoseEstimate estimate;

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            // Coordenadas relativas al origen de la alianza roja
            estimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
        } else {
            // Por defecto usamos azul (también aplica si la alianza no está definida aún)
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        // --- Publicar telemetría antes de los filtros de calidad ---
        // Así siempre vemos en el dashboard qué está detectando la cámara,
        // aunque no usemos la medición.
        int tagCount = (estimate != null) ? estimate.tagCount : 0;
        double avgTagDist = (estimate != null) ? estimate.avgTagDist : 0.0;

        SmartDashboard.putNumber("Vision/TagCount", tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist", avgTagDist);

        // --- Filtros de calidad: rechazar mediciones poco confiables ---

        // 1. La estimación no puede ser nula (sin datos de la cámara)
        if (estimate == null) {
            return;
        }

        // 2. Debe haber al menos un AprilTag visible para tener una estimación válida
        if (estimate.tagCount < 1) {
            return;
        }

        // 3. Rechazamos si el tag está demasiado lejos: la proyección 3D→2D
        //    acumula error rápidamente con la distancia.
        if (estimate.avgTagDist > MAX_TAG_DIST_METERS) {
            return;
        }

        // 4. El timestamp debe ser positivo. Un timestamp de 0 o negativo indica
        //    que la Limelight no tiene datos actuales o que hubo un error de red.
        if (estimate.timestampSeconds <= 0.0) {
            return;
        }

        // 5. La pose en sí no debe ser nula
        Pose2d pose = estimate.pose;
        if (pose == null) {
            return;
        }

        // --- Selección de desviaciones estándar según distancia al tag ---
        //
        // El filtro de Kalman interno de CTRE usa estos valores para ponderar
        // cuánto "mueve" la estimación hacia la medición de la cámara vs mantener
        // la odometría. Desviación alta = la cámara pesa menos.
        //
        var stdDevs = (estimate.avgTagDist <= NEAR_TAG_DIST_METERS)
            ? VecBuilder.fill(NEAR_STD_X, NEAR_STD_Y, NEAR_STD_THETA)
            : VecBuilder.fill(FAR_STD_X,  FAR_STD_Y,  FAR_STD_THETA);

        // --- Inyectar la medición al filtro de Kalman del drivetrain ---
        //
        // addVisionMeasurement() convierte el timestamp de FPGA a tiempo actual
        // internamente (ver CommandSwerveDrivetrain.java) y fusiona la pose
        // con la estimación de odometría existente.
        drivetrain.addVisionMeasurement(pose, estimate.timestampSeconds, stdDevs);
    }
}
