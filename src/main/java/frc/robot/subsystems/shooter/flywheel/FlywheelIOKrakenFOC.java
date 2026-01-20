package frc.robot.subsystems.shooter.flywheel;

import java.util.ArrayList;
import java.util.List;

import org.checkerframework.checker.units.qual.t;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;

public class FlywheelIOKrakenFOC implements FlywheelIO{
    private final TalonFX talon;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    private final NeutralOut neutralOut = new NeutralOut();
    private final double reduction;
    private final String name;

    public FlywheelIOKrakenFOC(
            int ID, CANBus bus, String name, int currentLimitAmps, boolean brake, double reduction) {
        this.reduction = reduction;
        this.name = name;
        this.talon = new TalonFX(ID,bus);
        

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast; 
        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
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

        @Override
        public void updateInputs(FlywheelIOInputs inputs){
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
        public void applyOutputs(FlywheelIOOutputs outputs){
            //TODO: implement this

        }
    }







