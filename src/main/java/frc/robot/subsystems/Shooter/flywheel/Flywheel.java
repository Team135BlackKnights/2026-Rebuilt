package frc.robot.subsystems.Shooter.flywheel;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

public class Flywheel extends SubsystemChecker {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();
    public static final LoggableTunedNumber kS = new LoggableTunedNumber("Flywheel/kS", 10,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kV = new LoggableTunedNumber("Flywheel/kV", .065,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kVMaxVelocity = new LoggableTunedNumber("Flywheel/kVMaxVelocity", 113,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kP = new LoggableTunedNumber("Flywheel/kP", 10,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kD = new LoggableTunedNumber("Flywheel/kD", 0,
            TuningConstants.isTuningShooter);
    private static final LoggableTunedNumber rateLimiter = new LoggableTunedNumber("Flywheel/SlewRateLimiter", 800,
            TuningConstants.isTuningShooter);
    SlewRateLimiter slewRateLimiter = new SlewRateLimiter(rateLimiter.get());

    public Flywheel(FlywheelIO io) {
        this.io = io;

        super.registerAllHardware(io.getSelfCheckingHardware());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        if (rateLimiter.hasChanged(hashCode())) {
            slewRateLimiter = new SlewRateLimiter(rateLimiter.get());
        }
        outputs.kP = kP.get();
        outputs.kD = kD.get();

        if (outputs.coast) {
            slewRateLimiter.reset(inputs.velocityRadsPerSec);
        }
        io.applyOutputs(outputs);

    }
    public void runVelocity(double velocityRadsPerSec) {
        outputs.velocityRadsPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
        outputs.feedForward = kS.get() * Math.signum(velocityRadsPerSec)
                + kV.get() * Math.min(velocityRadsPerSec, kVMaxVelocity.get());

        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
    }

    private void stop() {
        outputs.velocityRadsPerSec = 0.0;
        outputs.coast = false;
    }

    /** Returns the current velocity in radians per second. */
    public double getVelocity() {
        return inputs.velocityRadsPerSec;
    }

    public Command runTrackTargetCommand() {
        return runEnd(
                () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeed()),
                this::stop);
    }

    public Command runFixedCommand(DoubleSupplier velocity) {
        return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> tempMap = new HashMap<>();
        tempMap.put("Flywheel", inputs.tempCelsius);
        return tempMap;
    }


    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = io.getSelfCheckingHardware();
        for (SelfChecking motor : hardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        return inputs.supplyCurrentAmps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        io.setCurrentLimit(amps);
    }

    @Override
    protected Command systemCheckCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    }
}