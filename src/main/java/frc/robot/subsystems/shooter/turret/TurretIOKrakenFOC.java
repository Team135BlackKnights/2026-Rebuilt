package frc.robot.subsystems.Shooter.turret;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class TurretIOKrakenFOC implements TurretIO{

    private final TalonFX talon;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private static final double minTurretAngle = Units.degreesToRadians(-180);
    private static final double maxTurretAngle = Units.degreesToRadians(180);

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    private final NeutralOut neutralOut = new NeutralOut();

    private final double reduction;
    private final String name;

    public TurretIOKrakenFOC(int ID, CANBus bus, String name, int currentLimitAmps, boolean brake, double reduction){
        this.name = name;
        this.talon = new TalonFX(ID,bus);
        this.reduction = reduction;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        double minRotations = Units.radiansToRotations(minTurretAngle) * reduction;
        double maxRotations = Units.radiansToRotations(maxTurretAngle) * reduction;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotations;

        talon.getConfigurator().apply(config);
        
        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50, position,velocity,appliedVoltage,supplyCurrent,torqueCurrent,tempCelsius);
        talon.optimizeBusUtilization(0,1.0);


    }
    public void updateInputs(TurretIOInputs inputs){
        inputs.connected = BaseStatusSignal.refreshAll(position,velocity,appliedVoltage,supplyCurrent,torqueCurrent,tempCelsius)
            .isOK();
        inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }
    
    @Override
        public void setCurrentLimit(double amps){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimit = amps;
            talon.getConfigurator().apply(config);
        }

        @Override
        public List<SelfChecking> getSelfCheckingHardware() {
            List<SelfChecking> hardware = new ArrayList<SelfChecking>();
            hardware.add(new SelfCheckingTalonFX(name,talon));
            return hardware;
        }
        
        @Override
        public void applyOutputs(TurretIOOutputs outputs){
            PIDController pid = new PIDController(outputs.kP, 0, outputs.kD); //kP and kD
            talon.set(pid.calculate(outputs.velocityRadsPerSec/Turret.kVMaxVelocity.get()));
            pid.close();
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = outputs.coast ? NeutralModeValue.Coast : NeutralModeValue.Brake; //neutral mode
            talon.getConfigurator().apply(config);
        }
}