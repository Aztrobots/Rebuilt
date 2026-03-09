// Paquete
package frc.robot.subsystems;

// Importaciones
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

// Subsistema de la intake y el indexer del robot
public class IntakeSubsystem extends SubsystemBase {

    // Motores de la intake (NEO brushless vía SPARK MAX, ecosistema REV)
    public SparkMax intakeMotorLeft, intakeMotorRight;

    // Motor del indexer (TalonFX, ecosistema CTRE)
    public TalonFX indexerMotor;

    // Modos de control para el indexer
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);

    // Constructor
    public IntakeSubsystem() {
        // Motores de la intake (SPARK MAX con NEO)
        intakeMotorLeft  = new SparkMax(Ports.Intake.LEFT_SPARK_ID,  MotorType.kBrushless);
        intakeMotorRight = new SparkMax(Ports.Intake.RIGHT_SPARK_ID, MotorType.kBrushless);

        // El motor derecho gira en sentido opuesto al izquierdo
        intakeMotorRight.setInverted(true);

        // Indexer (TalonFX en bus RIO)
        indexerMotor = new TalonFX(Ports.Indexer.ID, Ports.RIO_BUS);
        configurarIndexer(indexerMotor, InvertedValue.Clockwise_Positive);
    }

    // Configuración del TalonFX del indexer
    private void configurarIndexer(TalonFX motor, InvertedValue invertDirection) {
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
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond))
            );

        motor.getConfigurator().apply(config);
    }

    // Detiene los motores de la intake
    public void stop() {
        intakeMotorLeft.set(0);
        intakeMotorRight.set(0);
    }

    // Control por voltaje porcentual para un motor TalonFX específico
    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    // Control por RPM con lazo PID (Slot0) para un motor TalonFX específico
    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    // Velocidad directa de ambos motores de la intake (rango -1 a 1)
    // TODO: velocidad de intake — ajustar si la nota no entra limpia (actualmente 0.75 en StartIntake)
    public void setSpeed(double speed) {
        intakeMotorLeft.set(speed);
        intakeMotorRight.set(speed);
    }

    public void setWristSpeed(double speed) {}

    // TODO: posicion abajo (0=home) y arriba (1=recoleccion) — medir en robot fisico con transportador
    public void setPosition(double position) {}

    public double getPosition() {
        return 0;
    }

    // Velocidad directa del indexer (rango -1 a 1)
    // TODO: velocidad del feeder para alimentar el shooter — calibrar segun nota (actualmente Constants.Shooter.FEEDER_SPEED)
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
