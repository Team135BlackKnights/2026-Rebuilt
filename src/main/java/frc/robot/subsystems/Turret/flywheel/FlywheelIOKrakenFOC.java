package frc.robot.subsystems.Turret.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.YAMS.mechanisms.FlyWheelConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.FlyWheel;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class FlywheelIOKrakenFOC implements FlywheelIO {
    private final SmartMotorControllerConfig config;
    private final TalonFX talon;
    private final SmartMotorController talonController;
    private final FlyWheelConfig shooterConfig;
    private final FlyWheel shooter;
    private final StatusSignal<AngularAcceleration> accel;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final double reduction;
    private final String name;

    public FlywheelIOKrakenFOC(
            CANBus bus, int ID, String name, int currentLimitAmps, double reduction, boolean invert, double maxRPM, double moi) {
        this.reduction = reduction;
        this.name = name;
        this.talon = new TalonFX(ID, bus);
        config = new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
            .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withMotorInverted(invert)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(currentLimitAmps))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));
        talonController = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), config);
        shooterConfig = new FlyWheelConfig(talonController)
            .withMOI(KilogramSquareMeters.of(moi))
            .withDiameter(Inches.of(4))
            .withUpperSoftLimit(RPM.of(maxRPM));
        shooter = new FlyWheel(shooterConfig);
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
        LinearVelocity linearSpeed = MetersPerSecond.of(motorVelocityRadsPerSec *Units.inchesToMeters(2));
        shooter.setMeasurementVelocitySetpoint(linearSpeed);

    }

    @Override
    public void setCurrentLimit(double amps) {
        talonController.setSupplyCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        talonController.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public void runVolts(double volts) {
        shooter.setVoltage(Volts.of(volts));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setPID(double p, double d, double ks, double kv, double ka) {
        talonController.setFeedback(p, 0, d);
        talonController.setFeedforward(ks, kv, ka, 0);
    }
@Override
    public void setRamp(double closedLoopRampSecs){
        talonController.setClosedLoopRampRate(Seconds.of(closedLoopRampSecs));
        talonController.setOpenLoopRampRate(Seconds.of(closedLoopRampSecs)); 
    }
    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingTalonFX(name + "_Flywheel", talon));
        return hardware;
    }
}
