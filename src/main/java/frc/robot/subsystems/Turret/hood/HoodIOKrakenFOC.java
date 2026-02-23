package frc.robot.subsystems.Turret.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;

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
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.YAMS.TalonFXSWrapper;
import frc.robot.utils.YAMS.mechanisms.ArmConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.Arm;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFXS;

public class HoodIOKrakenFOC implements HoodIO {
    private final String name;

    private final TalonFXS talon;
    private final CANcoder cancoder;

    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController motor;
    private final ArmConfig hoodConfig;
    private final Arm hood;

    private final double minAngleRads;
    private final double maxAngleRads;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    public HoodIOKrakenFOC(
            CANBus bus,
            int motorID,
            int cancoderID,
            String name,
            int currentLimitAmps,
            double rotorToEncoder,
            double encoderToArm,
            double minAngleRads,
            double maxAngleRads,
            double encoderOffset) {

        this.name = name;
        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;

        talon = new TalonFXS(motorID, bus);
        cancoder = new CANcoder(cancoderID, bus);

        final double rotorPerMechanism = rotorToEncoder * encoderToArm;

        motorConfig = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))
                .withSimClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))

                .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
                .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))

                .withGearing(new MechanismGearing(GearBox.fromReductionStages(rotorPerMechanism)))

                .withSoftLimit(Radians.of(minAngleRads), Radians.of(maxAngleRads))

                // Same-vendor absolute feedback (CTRE CANcoder)
                .withExternalEncoder(cancoder)
                .withExternalEncoderInverted(false)
                .withExternalEncoderGearing(encoderToArm)
                .withUseExternalFeedbackEncoder(true)
                .withExternalEncoderZeroOffset(Radians.of(encoderOffset))

                .withMotorInverted(false)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25));

        motor = new TalonFXSWrapper(talon, DCMotor.getMinion(1), motorConfig);

        hoodConfig = new ArmConfig(motor)
                .withLength(Inches.of(7))
                .withMOI(AdvancedMechanismConstants.Turret.hoodMOI)
                .withHardLimit(Radians.of(minAngleRads), Radians.of(maxAngleRads))
                .withStartingPosition(Radians.of(minAngleRads))
                ;
        ;

        hood = new Arm(hoodConfig);

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.name = name;
        inputs.connected = BaseStatusSignal.refreshAll(appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();

        inputs.positionRads = hood.getAngle().in(Radians);
        inputs.velocityRadsPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setPosition(double positionRads) {
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        hood.setMechanismPositionSetpoint(Radians.of(clamped));
    }

    @Override
    public void runVolts(double volts) {
        hood.setVoltage(Volts.of(volts));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setCurrentLimit(double amps) {
        motor.setSupplyCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public void setPID(double p, double d, double ks, double kv) {
        motor.setFeedback(p, 0.0, d);
        motor.setFeedforward(ks, kv, 0.0, 0.0);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.add(new SelfCheckingTalonFXS(name + "_Hood", talon));
        return hardware;
    }
}