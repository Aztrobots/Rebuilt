package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX bwMotor, twMotor, rsMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);
    private final PositionVoltage posvol = new PositionVoltage(0);

    public ShooterSubsystem() {
        bwMotor = new TalonFX(0);
        twMotor = new TalonFX(0);
        //lsMotor = new TalonFX(0);
        rsMotor = new TalonFX(0);
        motors = List.of(bwMotor, twMotor, rsMotor);

        configureMotor(bwMotor, InvertedValue.Clockwise_Positive);
        configureMotor(twMotor, InvertedValue.Clockwise_Positive);
        //configureMotor(lsMotor, null);
        configureMotor(rsMotor, InvertedValue.CounterClockwise_Positive);

        bwMotor.setControl(new Follower(twMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        //lsMotor.setControl(new Follower(rsMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
            /* .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / RPM.of(6000).in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );*/
        
        motor.getConfigurator().apply(config);
    }

    public void stop() {
        for (final TalonFX motor : motors) motor.stopMotor();
    }

    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    public void setWristPosition(double position) {
        twMotor.setControl(posvol.withPosition(position));
    }

    public double getWristPosition() {
        return twMotor.getPosition().getValueAsDouble();
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velVol);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velVol.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, RPM.of(100));
        });
    }
}
