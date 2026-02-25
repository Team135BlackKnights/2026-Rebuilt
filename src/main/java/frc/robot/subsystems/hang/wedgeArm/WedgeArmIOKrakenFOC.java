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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
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
    protected final Servo servo;
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
    protected boolean holdingServo = false;
    protected double tolDeg = 2.0;
    protected static final double SERVO_MOVE_DELAY_SECS = 0.25;
    protected final Timer servoDelayTimer = new Timer();
    protected boolean waitingForServoDelay = false;
    protected Double pendingSetpointRads = null;
    protected Double lastCommandedSetpointRads = null;
    public WedgeArmIOKrakenFOC(
            CANBus bus,
            int motorID,
            int servoID,
            String name,
            int currentLimitAmps,
            boolean invert,
            boolean brake,
            double reduction) {
        this.name = name;

        talon = new TalonFX(motorID, bus);
        servo = new Servo(servoID);
        servo.setBoundsMicroseconds(2500,0,0,0,500); //1500 center?
        motorConfig = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90))
                .withSimClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(90))
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

        // If servo is locked, arm must not move at all.
        if (servo.get() <= 0.0) {
            wedgeArm.setVoltage(Volts.of(0.0));
            waitingForServoDelay = false;
        } else {
            // Respect servo travel time before allowing arm movement.
            if (waitingForServoDelay) {
                wedgeArm.setVoltage(Volts.of(0.0));
                if (servoDelayTimer.hasElapsed(SERVO_MOVE_DELAY_SECS)) {
                    waitingForServoDelay = false;
                }
            }

            if (!waitingForServoDelay && pendingSetpointRads != null) {
                wedgeArm.setMechanismPositionSetpoint(Radians.of(pendingSetpointRads));
            }

            // Lock servo when arm is within tolerance of target.
            if (pendingSetpointRads != null
                    && Math.abs(pendingSetpointRads - wedgeArm.getAngle().in(Radians))
                            <= Math.toRadians(tolDeg)) {
                holdingServo = true;
                servo.setPosition(0.0);
                wedgeArm.setVoltage(Volts.of(0.0));
            }
        }

        inputs.name = name;
        inputs.positionRads = wedgeArm.getAngle().in(Radians);
        inputs.velocityRadsPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        inputs.servoPos = servo.getPosition();
        inputs.servoHold = holdingServo;
    }

    @Override
    public void setPosition(double positionRads) {
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        if (lastCommandedSetpointRads != null
                && Math.abs(clamped - lastCommandedSetpointRads) < 1e-6) {
            return;
        }
        lastCommandedSetpointRads = clamped;
        pendingSetpointRads = clamped;
        holdingServo = false;
        servo.setPosition(.2);
        waitingForServoDelay = true;
        servoDelayTimer.restart();
        wedgeArm.setVoltage(Volts.of(0.0));
    }
    @Override
    public void setVoltage(double volts) {
        if (servo.get() <= 0.0 || waitingForServoDelay) {
            wedgeArm.setVoltage(Volts.of(0.0));
            return;
        }
        wedgeArm.setVoltage(Volts.of(volts));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
        holdingServo = true;
        waitingForServoDelay = false;
        lastCommandedSetpointRads = null;
        pendingSetpointRads = null;
        servo.setPosition(0.0);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv,double tolDeg) {
        motor.setFeedback(p, i, d);
        motor.setFeedforward(ks, kv, 0.0, 0.0);
        this.tolDeg = tolDeg;
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
