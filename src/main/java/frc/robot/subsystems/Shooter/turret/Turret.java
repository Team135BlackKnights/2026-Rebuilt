package frc.robot.subsystems.Shooter.turret;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.subsystems.Shooter.turret.TurretIOInputsAutoLogged;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

public class Turret extends SubsystemChecker {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIOOutputs outputs = new TurretIOOutputs();
    public static final LoggableTunedNumber kVMaxVelocity = new LoggableTunedNumber("Turret/kVMaxVelocity", 113,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kP = new LoggableTunedNumber("Turret/kP", 10,
            TuningConstants.isTuningShooter);
    public static final LoggableTunedNumber kD = new LoggableTunedNumber("Turret/kD", 0,
            TuningConstants.isTuningShooter);
    private static final LoggableTunedNumber rateLimiter = new LoggableTunedNumber("Turret/SlewRateLimiter", 800,
            TuningConstants.isTuningShooter);
    SlewRateLimiter slewRateLimiter = new SlewRateLimiter(rateLimiter.get());

    public Turret(TurretIO io) {
        this.io = io;

        super.registerAllHardware(io.getSelfCheckingHardware());
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

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


        // Log Turret setpoint
        Logger.recordOutput("Turret/Setpoint", outputs.velocityRadsPerSec);
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
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
        //TODO implement this
    }

    public Command runFixedCommand(DoubleSupplier velocity) {
        return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> tempMap = new HashMap<>();
        tempMap.put("Turret", inputs.tempCelsius);
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