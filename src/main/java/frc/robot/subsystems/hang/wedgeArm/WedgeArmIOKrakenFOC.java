package frc.robot.subsystems.hang.wedgeArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.YAMS.mechanisms.ArmConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.Arm;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class WedgeArmIOKrakenFOC implements WedgeArmIO {
    protected final String name;

    protected final TalonFX talon;
    protected final SmartMotorControllerConfig motorConfig;
    protected final SmartMotorController motor;
    protected final ArmConfig wedgeArmConfig;
    protected final Arm wedgeArm;

    protected final double minAngleRads = SimpleMechanismConstants.Climber.wedgeMinAngleRads;
    protected final double maxAngleRads = SimpleMechanismConstants.Climber.wedgeMaxAngleRads;

    protected final StatusSignal<Voltage> appliedVoltage;
    protected final StatusSignal<Current> supplyCurrent;
    protected final StatusSignal<Current> torqueCurrent;
    protected final StatusSignal<Temperature> tempCelsius;

    public WedgeArmIOKrakenFOC(
            CANBus bus,
            int motorID,
            String name,
            int currentLimitAmps,
            boolean invert,
            boolean brake,
            double reduction) {
        this.name = name;

        talon = new TalonFX(motorID, bus);

        motorConfig = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(720))
                .withSimClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(720))
                .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
                .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(reduction)))
                .withMotorInverted(invert)
                .withIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withSupplyCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopControlPeriod(Milliseconds.of(10))
                .withClosedLoopRampRate(Seconds.of(0.0))
                .withOpenLoopRampRate(Seconds.of(0.0));

        motor = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), motorConfig);

        wedgeArmConfig = new ArmConfig(motor)
                .withGravity(true)
                .withLength(Inches.of(7))
                .withMOI(KilogramSquareMeters.of(.0010)) //its only for sim, idc.
                .withHardLimit(Radians.of(minAngleRads), Radians.of(maxAngleRads))
                .withStartingPosition(Radians.of(minAngleRads));
        wedgeArm = new Arm(wedgeArmConfig);

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(WedgeArmIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.positionRads = wedgeArm.getAngle().in(Radians);
        inputs.velocityRadsPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setPosition(double positionRads) {
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        wedgeArm.setMechanismPositionSetpoint(Radians.of(clamped));
    }

    @Override
    public void setVoltage(double volts) {
        wedgeArm.setVoltage(Volts.of(volts));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv) {
        motor.setFeedback(p, i, d);
        motor.setFeedforward(ks, kv, 0.0, 0.0);
    }

    @Override
    public void setCurrentLimit(double amps) {
        motor.setSupplyCurrentLimit(Amps.of(amps));
        motor.setStatorCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return List.of(new SelfCheckingTalonFX(name, talon));
    }
}
