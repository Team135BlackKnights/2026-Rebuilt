package frc.robot.subsystems.Turret.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.YAMS.mechanisms.FlyWheelConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.FlyWheel;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;

public class FlywheelIOSim implements FlywheelIO {
    private final SmartMotorControllerConfig config;
    private final SmartMotorController controller;
    private final FlyWheelConfig shooterConfig;
    private final FlyWheel shooter;
    private final double reduction;
    @SuppressWarnings("unused")
    private final String name;

    public FlywheelIOSim(
            CANBus bus, int ID, String name, int currentLimitAmps, double reduction, boolean invert, double maxRPM,
            double moi) {
        this.reduction = reduction;
        this.name = name;
        TalonFX talon = new TalonFX(ID, bus);
        config = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.2, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
        .withSimClosedLoopController(0.2, 0, 0, DegreesPerSecond.of(0), DegreesPerSecondPerSecond.of(0))
        .withFeedforward(new SimpleMotorFeedforward(0, 0.02, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.02, 0))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(reduction)))
                .withMotorInverted(invert)
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25));
        controller = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), config);
        shooterConfig = new FlyWheelConfig(controller)
                .withMOI(KilogramSquareMeters.of(moi))
                .withDiameter(Inches.of(4))
                .withUpperSoftLimit(RPM.of(maxRPM));
        shooter = new FlyWheel(shooterConfig);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        shooter.simIterate();
        //shooter.getMechanismSetpointVelocity().ifPresent(setpoint -> System.out.println("Flywheel setpoint: " + setpoint.in(RadiansPerSecond) + " rad/s, currently " + shooter.getSpeed().in(RadiansPerSecond) + " rad/s"));
        inputs.connected = true;
        inputs.accelRadsPerSec2 = 0;
        inputs.velocityRadsPerSec = shooter.getSpeed().in(RadiansPerSecond);
        inputs.appliedVoltage = controller.getVoltage().in(Volts);
        inputs.supplyCurrentAmps = controller.getStatorCurrent().in(Amps);
        inputs.torqueCurrentAmps = controller.getStatorCurrent().in(Amps);
        inputs.tempCelsius = controller.getTemperature().in(Celsius);
    }

    @Override
    public void setVelocity(double velocityRadsPerSec) {
        double motorVelocityRadsPerSec = velocityRadsPerSec * reduction;
        //get linear speed at this rads per sec of the flywheel
        LinearVelocity linearSpeed = MetersPerSecond.of(motorVelocityRadsPerSec *Units.inchesToMeters(2));
        shooter.setMeasurementVelocitySetpoint(linearSpeed);

    }

    @Override
    public void setCurrentLimit(double amps) {
        controller.setSupplyCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        controller.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
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
        controller.setFeedback(p, 0, d);
        controller.setFeedforward(ks, kv, ka, 0);
    }
    @Override
    public void setRamp(double closedLoopRampSecs){
        controller.setClosedLoopRampRate(Seconds.of(closedLoopRampSecs));
        controller.setOpenLoopRampRate(Seconds.of(closedLoopRampSecs)); 
    }
}
