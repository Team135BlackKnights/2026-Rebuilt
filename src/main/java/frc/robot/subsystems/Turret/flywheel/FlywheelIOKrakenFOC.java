package frc.robot.subsystems.Turret.flywheel;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class FlywheelIOKrakenFOC implements FlywheelIO {
    private final TalonFX talon;
    private final StatusSignal<AngularAcceleration> accel;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
    private final double reduction;
    private final String name;

    public FlywheelIOKrakenFOC(
            CANBus bus, int ID, String name, int currentLimitAmps,double reduction) {
        this.reduction = reduction;
        this.name = name;
        this.talon = new TalonFX(ID, bus);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        talon.getConfigurator().apply(config);
        accel = talon.getAcceleration();
        velocity = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(50, accel, velocity, appliedVoltage, supplyCurrent, torqueCurrent,
                tempCelsius);

        talon.optimizeBusUtilization(0, 1.0);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.connected = BaseStatusSignal
                .refreshAll(accel, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.accelRadsPerSec2 = Units.rotationsToRadians(accel.getValueAsDouble()) / reduction;
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRadsPerSec) {
        double motorVelocityRadsPerSec = velocityRadsPerSec * reduction;
        velocityControl.withVelocity(Units.radiansToRotations(motorVelocityRadsPerSec));
        talon.setControl(velocityControl);
    }

    @Override
    public void setCurrentLimit(double amps) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(config);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talon.getConfigurator().apply(config);
    }

    @Override
    public void runVolts(double volts) {
        voltageOut.withOutput(volts);
        talon.setControl(voltageOut);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setPID(double p, double d, double ks, double kv) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = p;
        config.Slot0.kD = d;
        config.Slot0.kS = ks;
        config.Slot0.kV = kv;
        talon.getConfigurator().apply(config);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingTalonFX(name + "_Flywheel", talon));
        return hardware;
    }
}
