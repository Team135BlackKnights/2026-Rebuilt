package frc.robot.subsystems.intake.arm;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class ArmIOKrakenFOC implements ArmIO {
    private final TalonFX motor;
    private final double reduction;
    private final String name;
    private final PositionTorqueCurrentFOC control = new PositionTorqueCurrentFOC(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    public ArmIOKrakenFOC(CANBus bus,int motorID, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        this.motor = new TalonFX(motorID, bus);
        this.reduction = reduction;
        this.name = name;
        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        motor.getConfigurator().apply(config);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        tempCelsius = motor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

        motor.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void setPosition(double positionRads) {
        control.withPosition(positionRads * reduction);
        motor.setControl(control);
    }

    @Override
    public void setVoltage(double volts) {
        voltageControl.withOutput(volts);
        motor.setControl(voltageControl);

    }

    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(

                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.positionRads = position.getValueAsDouble() / reduction;
        inputs.velocityRadsPerSec = velocity.getValueAsDouble() / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        config.Slot0.kS = ks;
        config.Slot0.kV = kv;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.CurrentLimits.SupplyCurrentLimit = amps;
        config.TorqueCurrent.PeakForwardTorqueCurrent = amps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -amps;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.getConfigurator().apply(config);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return List.of(new SelfCheckingTalonFX(name, motor));
    }

}
