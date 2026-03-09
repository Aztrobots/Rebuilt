package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Ports;
import frc.robot.Constants.Mechanisms;

public class ShooterSubsystem extends SubsystemBase {

    // Ruedas de disparo — TalonFX en bus RIO. rsMotor es maestro; lsMotor sigue como Follower.
    // public final requerido: StartShooter.java accede a rsMotor directamente para setRPM().
    public final TalonFX rsMotor = new TalonFX(Ports.Shooter.RIGHT_ID, Ports.RIO_BUS);
    public final TalonFX lsMotor = new TalonFX(Ports.Shooter.LEFT_ID,  Ports.RIO_BUS);

    // Pivote del hood — Kraken mini (TalonFX). twMotor es maestro; bwMotor sigue como Follower opuesto.
    // En FTC sería un servo de posición; aquí es un TalonFX con MotionMagicVoltage en lazo cerrado.
    public final TalonFX twMotor = new TalonFX(Ports.Hood.LEFT_ID,  Ports.RIO_BUS);
    public final TalonFX bwMotor = new TalonFX(Ports.Hood.RIGHT_ID, Ports.RIO_BUS);

    // VelocityVoltage: control de velocidad en lazo cerrado para las ruedas de disparo.
    private final VelocityVoltage   velVol     = new VelocityVoltage(0).withSlot(0);
    // VoltageOut: control en lazo abierto (porcentaje de voltaje), usado en set() y setPercentOutput().
    private final VoltageOut        volOut     = new VoltageOut(0);
    // MotionMagicVoltage: perfil de movimiento trapezoidal para el pivote del hood.
    // Equivalente a RUN_TO_POSITION de FTC pero con límites de velocidad y aceleración configurables.
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    // Solo el maestro del shooter; lsMotor (follower) no recibe comandos directos.
    // isVelocityWithinTolerance() solo tiene sentido verificar el motor maestro.
    private final List<TalonFX> shooterMotors;

    // Todos los motores para stop() — detiene tanto ruedas como hood.
    private final List<TalonFX> allMotors;

    public ShooterSubsystem() {
        shooterMotors = List.of(rsMotor);
        allMotors     = List.of(rsMotor, lsMotor, twMotor, bwMotor);

        // Ruedas de disparo: configurar solo el maestro; follower hereda el comportamiento.
        configureShooterMotor(rsMotor, InvertedValue.CounterClockwise_Positive);
        lsMotor.setControl(new Follower(rsMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        // Hood: configurar maestro con MotionMagic; follower opuesto por simetría del mecanismo.
        configureHoodMotor(twMotor, InvertedValue.Clockwise_Positive);
        bwMotor.setControl(new Follower(twMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Config para ruedas de disparo (flywheel). Control de velocidad puro — sin compensación
     * de gravedad (KG/GravityType no aplica a masas giratorias). PeakReverseVoltage NO se
     * restringe a 0: el follower necesita voltaje negativo para girar en sentido Aligned.
     */
    private void configureShooterMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                // TODO: tunar kP y kV con SysId antes de competencia — valores actuales son punto de partida
                new Slot0Configs()
                    .withKP(0.11)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
                    .withKS(0.2)
            );

        motor.getConfigurator().apply(config);
    }

    /**
     * Config para el pivote del hood (Kraken mini). Usa MotionMagicVoltage con perfil
     * trapezoidal. Brake en neutro: el pivote no debe caer cuando el comando termina.
     * KG + Arm_Cosine compensan el torque gravitacional en función del ángulo actual,
     * igual que un feedforward de coseno en FTC.
     *
     * TODO: tune con SysId en robot físico — kP, kV, cruiseVelocity y acceleration son
     * placeholders calculados conservadoramente para la primera prueba.
     */
    private void configureHoodMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Kraken mini: límites más bajos que el shooter; mecanismo de posición, no de velocidad.
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10.0)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12)
                    .withKS(0.0)
                    .withKG(1.0)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    // Velocidad de crucero y aceleración en rot/s y rot/s² respectivamente.
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(80))
                    .withMotionMagicAcceleration(160)
                    .withMotionMagicJerk(0)
            );

        motor.getConfigurator().apply(config);
    }

    /** Temperatura más alta entre los cuatro motores del subsistema. */
    public double getShooterTemp() {
        return allMotors.stream()
            .mapToDouble(m -> m.getDeviceTemp().getValueAsDouble())
            .max()
            .orElse(0.0);
    }

    /** Detiene todos los motores (ruedas + hood). */
    public void stop() {
        for (final TalonFX motor : allMotors) motor.stopMotor();
    }

    /** Control en lazo abierto por voltaje. Útil para pruebas manuales. */
    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    /** Dispara las ruedas a porcentaje de voltaje (lazo abierto). */
    public void set(double speed) {
        rsMotor.set(speed);
    }

    /** Mueve el hood a velocidad manual (lazo abierto). Usado durante calibración. */
    public void setWristSpeed(double speed) {
        twMotor.set(speed);
    }

    /**
     * Control de velocidad en lazo cerrado para las ruedas de disparo.
     * Se pasa el motor explícitamente para permitir control individual (ej. solo rsMotor).
     */
    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    /**
     * Mueve el hood a una posición angular (rotaciones del rotor) usando MotionMagicVoltage.
     * El perfil trapezoidal limita velocidad y aceleración según configureHoodMotor().
     *
     * Posiciones esperadas (todas requieren medición en robot fisico):
     *   WRIST_ANGLE_SHOOT — TODO: angulo de disparo normal — medir en robot fisico con transportador
     *   WRIST_ANGLE_HOME  — TODO: posicion de reposo del wrist — verificar que no choque con intake
     */
    public void setWristPosition(double position) {
        twMotor.setControl(motionMagic.withPosition(position));
    }

    /** Posición actual del rotor de twMotor en rotaciones. */
    public double getWristPosition() {
        return twMotor.getPosition().getValueAsDouble();
    }

    /**
     * Calcula RPM de disparo interpolando una tabla empírica de distancia→RPM.
     * Llenar los puntos reales antes de usar en competencia.
     */
    public double calculateRPMWithITM(String limelightName, double distance) {
        final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        // TODO: reemplazar con puntos empíricos reales (distancia en metros → RPM)
        // table.put(1.5, 2500.0);
        // table.put(2.5, 3200.0);
        // table.put(4.0, 4100.0);
        table.put(0.0, 0.0);

        return table.get(distance);
    }

    /**
     * Calcula RPM de disparo analíticamente con cinemática de proyectil.
     * Requiere constantes de hardware correctas en Constants.Mechanisms.
     */
    public double calculateShooterRPM(
        String limelightName,
        double cameraHeight,
        double cameraMountingAngle,
        double hubHeight
    ) {
        double targetAngle = LimelightHelpers.getTY(limelightName);

        double distanceToTarget = (1.8288 - cameraHeight)
            / Math.tan((cameraMountingAngle + targetAngle) * (Math.PI / 180.0));

        double velocity = Math.sqrt(
            (9.81 * (distanceToTarget * distanceToTarget))
            / (2 * Math.pow(Math.cos(Mechanisms.shooterMountingAngle), 2)
                * (distanceToTarget * Math.tan(Mechanisms.shooterMountingAngle)
                    - (hubHeight - Mechanisms.shooterMountingHeight)))
        );

        return velocity / (2 * Math.PI * Mechanisms.shooterWheelRadius) * 60;
    }

    /**
     * Verdadero si el motor maestro del shooter está en modo velocidad y dentro de
     * ±150 RPM del setpoint. Tolerancia de 150 RPM: 100 RPM causaba stutter al disparar.
     */
    public boolean isVelocityWithinTolerance() {
        return shooterMotors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velVol);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velVol.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, RPM.of(150));
        });
    }
}
