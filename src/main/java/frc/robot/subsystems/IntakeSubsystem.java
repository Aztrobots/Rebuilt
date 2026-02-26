//Package
package frc.robot.subsystems;

//Imports
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//Create class
public class IntakeSubsystem extends SubsystemBase {

    //Atributes
    public final TalonFX liMotor, riMotor, intakeMotor;
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);
    private final PositionVoltage posvol = new PositionVoltage(0);

    //Constructor
    public IntakeSubsystem() {
        liMotor = new TalonFX(0);
        riMotor = new TalonFX(0);
        intakeMotor = new TalonFX(0);

        configureMotor(liMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(riMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(intakeMotor, InvertedValue.Clockwise_Positive);
        liMotor.setControl(new Follower(riMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    //Consfigure motor method
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

    //Another methods
    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    public void setSpeed(double speed, TalonFX motor) {
        motor.set(speed);
    }

    public void setPosition(double position) {
        riMotor.setControl(posvol.withPosition(position));
    }

    public double getPosition() {
        return riMotor.getPosition().getValueAsDouble();
    }

    public void startFeeder() {
        setRPM(2000, intakeMotor);
    }
}
