package frc.robot.subsystems.Turret.hood;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class HoodIOKrakenFOC implements HoodIO {
    private final TalonFX talon;
    private final CANcoder encoder;
    private final TalonFXConfiguration config;
    private final String name;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0.0);

    public HoodIOKrakenFOC(CANBus bus, int ID, int encoderID, String name, int currentLimitAmps, double rotorToEncoder, double encoderToArm, double minAngleRads, double maxAngleRads) {
        this.talon = new TalonFX(ID, bus);
        this.encoder = new CANcoder(encoderID,bus);
        this.name = name;
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = rotorToEncoder;    
        config.Feedback.SensorToMechanismRatio = encoderToArm;
        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(minAngleRads - Units.degreesToRadians(1.0)); //trying to go to 11 deg
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(maxAngleRads + Units.degreesToRadians(1.0)); //trying to go to 39 deg 
        talon.getConfigurator().apply(config);
        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();

        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent,
                tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);
    }
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.name = this.name;
        inputs.connected = talon.isConnected();
        inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }
    @Override
    public void setPosition(double positionRads) {
        positionControl.withPosition(Units.radiansToRotations(positionRads));
        talon.setControl(positionControl);
    }
    @Override
    public void runVolts(double volts) {
        voltageOut.withOutput(volts);
        talon.setControl(voltageOut);
    }
    @Override
    public void stop() {
        talon.stopMotor();
    }
    @Override
    public void setCurrentLimit(double amps) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(config);
    }
    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        talon.getConfigurator().apply(config);
    }
    @Override
    public void setPID(double p, double d, double ks, double kv) {
        config.Slot0.kP = p;
        config.Slot0.kD = d;
        config.Slot0.kS = ks;
        config.Slot0.kV = kv;
        talon.getConfigurator().apply(config);
    }
    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingTalonFX(name+"_Hood", talon));
        return hardware;
    }

}
