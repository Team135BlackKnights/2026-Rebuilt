package frc.robot.subsystems.intake.frontRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.IntakeConstants;
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

public class FrontRollersIOKrakenFOC implements FrontRollersIO {
    private final SmartMotorControllerConfig config;
    private final TalonFX talon;
    private final SmartMotorController controller;
    private final FlyWheelConfig rollerConfig;
    private final FlyWheel roller;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0);
    private final double reduction;
    private final String name;

    public FrontRollersIOKrakenFOC(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        this.reduction = reduction;
        this.name = name;
        talon = new TalonFX(motorID, bus);

        config = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
                .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
                .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                .withMotorInverted(invert)
                .withIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25));

        controller = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), config);
        rollerConfig = new FlyWheelConfig(controller)
                .withMOI(KilogramSquareMeters.of(IntakeConstants.frontRollersMOI))
                .withDiameter(Meters.of(IntakeConstants.rollersDiameterMeters));
        roller = new FlyWheel(rollerConfig);

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
    public void updateInputs(GenericRollerSystemIOInputs inputs) {
        inputs.connected = BaseStatusSignal
                .refreshAll(position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocityRadsPerSec) {
        double motorVelocityRadsPerSec = velocityRadsPerSec * reduction;
        LinearVelocity linearSpeed =
                MetersPerSecond.of(motorVelocityRadsPerSec * (IntakeConstants.rollersDiameterMeters / 2.0));
        if (!controller.isClosedLoopRunning()) {
            controller.startClosedLoopController();
        }
        roller.setMeasurementVelocitySetpoint(linearSpeed);
    }

    @Override
    public void setCurrentLimit(double amps) {
        controller.setSupplyCurrentLimit(Amps.of(amps));
    }

    public void setBrakeMode(boolean brake) {
        controller.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public void runVolts(double volts) {
        roller.setVoltage(Volts.of(volts));
    }

    @Override
    public void runCurrent(double amperes) {
        talon.setControl(currentControl.withOutput(amperes));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setPID(double p, double d, double ks, double kv, double ka) {
        controller.setFeedback(p, 0, d);
        controller.setFeedforward(ks, kv, ka, 0);
    }

    @Override
    public void setRamp(double closedLoopRampSecs) {
        controller.setClosedLoopRampRate(Seconds.of(closedLoopRampSecs));
        controller.setOpenLoopRampRate(Seconds.of(closedLoopRampSecs));
    }

    @Override
    public boolean supportsVelocityControl() {
        return true;
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingTalonFX(name, talon));
        return hardware;
    }
}
