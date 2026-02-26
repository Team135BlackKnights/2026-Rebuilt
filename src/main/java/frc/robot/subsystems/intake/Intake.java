package frc.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.intake.frontRollers.FrontRollers;
import frc.robot.subsystems.intake.indexer.Indexer;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;
import lombok.Getter;

public class Intake extends SubsystemChecker {

    // Tuning
    private static final LoggableTunedNumber arm_kP = new LoggableTunedNumber("Intake/Arm/kP", 10,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kI = new LoggableTunedNumber("Intake/Arm/kI", 0.0,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kD = new LoggableTunedNumber("Intake/Arm/kD", 1,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kS = new LoggableTunedNumber("Intake/Arm/kS", 1.7,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kV = new LoggableTunedNumber("Intake/Arm/kV", 0.0,
            TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber arm_kG = new LoggableTunedNumber("Intake/Arm/kG", 3,
            TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber arm_motionSpeed = new LoggableTunedNumber("Intake/Arm/MotionCruiseRadPerSec",999,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_motionAccel = new LoggableTunedNumber
            ("Intake/Arm/MotionAccelRadPerSec2", 999, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_motionJerk = new LoggableTunedNumber
            ("Intake/Arm/MotionJerkRadPerSec3", 999, TuningConstants.isTuningIntake);
    // Setpoints
    private static final LoggableTunedNumber angle_stow = new LoggableTunedNumber("Intake/Setpoints/StowRads",
            1.25, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber angle_ground = new LoggableTunedNumber("Intake/Setpoints/GroundRads", 0.0,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber time_jackhammer = new LoggableTunedNumber("Intake/JackhammerTimeSecs",
            .25, TuningConstants.isTuningIntake);

    // Tolerance
    private static final LoggableTunedNumber arm_tolerance = new LoggableTunedNumber("Intake/ToleranceRads", 0.2,
            TuningConstants.isTuningIntake);

    // IO
    private final ArmIO armIO;
    private final Indexer indexer;
    private final FrontRollers frontRollers;

    // Inputs
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    protected Goal lastGoal;
    private final Timer stateTimer = new Timer();

    // State Machine
    public enum Goal {
        START, // Initial state
        STOW, // Arm up, rollers slow
        INTAKE_GROUND, // Arm down, rollers intake
        INTAKE_OUTER_IDLE, // Arm down, rollers idling
        JACKHAMMERING_OUT, // Rapidly pulse rollers to dislodge jams (with arm down)
        JACKHAMMERING_IN, // Rapidly pulse rollers to dislodge jams (with arm up)
        SHOOTING, // Don't mess with the arm, but run the rollers at shooting speed
        TUNING,
    }

    private Goal goal = Goal.START;
    private double currentArmSetpoint = 0.0;
    private double currentRollerVolts = 0.0;
    @Getter
    private boolean intakeDeployed = false;
    public Intake(ArmIO armIO, Indexer indexer, FrontRollers frontRollers) {
        this.armIO = armIO;
        this.indexer = indexer;
        this.frontRollers = frontRollers;

        // Apply initial PID
        updateTunablePIDs();
    }

    @Override
    public void periodic() {
        // 1. Update Inputs
        armIO.updateInputs(armInputs);
        Logger.processInputs("Intake/Arm", armInputs);
        indexer.periodic();
        frontRollers.periodic();

        // 2. Check for Tuning Updates
        updateTunablePIDs();

        // 3. Safety Check
        if (DriverStation.isDisabled()) {
            goal = Goal.STOW; // Reset state on disable
            armIO.stop();
            indexer.setGoal(Indexer.Goal.STOPPED);
            frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            return;
        }

        if (getGoal() != lastGoal) {
            stateTimer.reset();
            lastGoal = getGoal();
        }
        // 4. State Machine Logic
        switch (goal) {
            case TUNING -> {
                indexer.setGoal(Indexer.Goal.STOPPED);
                frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            }
            case START -> {
                currentArmSetpoint = angle_stow.get();
                indexer.setGoal(Indexer.Goal.STOPPED);
                frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            }
            case STOW -> {
                currentArmSetpoint = angle_stow.get();
                indexer.setGoal(Indexer.Goal.STOPPED);
                frontRollers.setGoal(FrontRollers.Goal.IDLING);
            }
            case INTAKE_GROUND -> {
                currentArmSetpoint = angle_ground.get();
                indexer.setGoal(Indexer.Goal.STOPPED);
                frontRollers.setGoal(FrontRollers.Goal.INTAKING);
            }
            case INTAKE_OUTER_IDLE -> {
                currentArmSetpoint = angle_ground.get();
                indexer.setGoal(Indexer.Goal.STOPPED);
                frontRollers.setGoal(FrontRollers.Goal.IDLING);
            }
            case JACKHAMMERING_IN -> {
                currentArmSetpoint = angle_stow.get();
                //if the timer is in the first half of the jackhammer time, set to jackhammer in, else out
                double t = Timer.getFPGATimestamp() % (time_jackhammer.get() * 2);
                if (t < time_jackhammer.get()) {
                    indexer.setGoal(Indexer.Goal.JACKHAMMER_IN);
                    frontRollers.setGoal(FrontRollers.Goal.INTAKING);
                } else {
                    indexer.setGoal(Indexer.Goal.JACKHAMMER_OUT);
                    frontRollers.setGoal(FrontRollers.Goal.INTAKING);
                }
            }
            case JACKHAMMERING_OUT -> {
                currentArmSetpoint = angle_ground.get();
                double t = Timer.getFPGATimestamp() % (time_jackhammer.get() * 2);
                if (t < time_jackhammer.get()) {
                    indexer.setGoal(Indexer.Goal.JACKHAMMER_OUT);
                    frontRollers.setGoal(FrontRollers.Goal.INTAKING);
                } else {
                    indexer.setGoal(Indexer.Goal.JACKHAMMER_IN);
                    frontRollers.setGoal(FrontRollers.Goal.INTAKING);
                }
            }
            case SHOOTING -> {
                // Don't mess with the arm position, since we might want to shoot from either stow or ground intake
                indexer.setGoal(Indexer.Goal.SHOOTING);
                frontRollers.setGoal(FrontRollers.Goal.SHOOTING);
            }

        }
        if (goal != Goal.TUNING)
            armIO.setPosition(currentArmSetpoint);

        // 6. Logging
        Logger.recordOutput("Intake/Goal", goal);
        Logger.recordOutput("SuperStructure/Intake/IntakeGoal", goal);
        Logger.recordOutput("Intake/SetpointAngle", currentArmSetpoint);
        Logger.recordOutput("Intake/SetpointVolts", currentRollerVolts);
        Logger.recordOutput("Intake/AtSetpoint", isAtSetpoint());
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
        if (goal == Goal.INTAKE_GROUND || goal == Goal.INTAKE_OUTER_IDLE) {
            intakeDeployed = true;
        } else if (goal == Goal.STOW) {
            intakeDeployed = false;
        }
    }
    public void zero(){
        armIO.zero();
    }
    public Goal getGoal() {
        return goal;
    }
    public void runCharacterization(double volts) {
        goal = Goal.TUNING;
        armIO.setVoltage(volts);
    }
    public double getCharacterizationMeasurement() {
        return armInputs.positionRads;
    }
    public double getCharVelocity() {
        return armInputs.velocityRadsPerSec;
    }
    public boolean isAtSetpoint() {
        return Math.abs(armInputs.positionRads - currentArmSetpoint) < arm_tolerance.get();
    }
    public double getArmAngle() {
        return armInputs.positionRads;
    }
    private boolean isArmConnected() {
        return armInputs.connected;
    }
    private boolean isIndexerConnected() {
        return indexer.isConnected();
    }
    private boolean isFrontRollersConnected() {
        return frontRollers.isConnected();
    }
    public boolean isFullyConnected() {
        return isArmConnected() && isIndexerConnected() && isFrontRollersConnected();
    }
    private void updateTunablePIDs() {
        if (arm_kP.hasChanged(hashCode()) || arm_kI.hasChanged(hashCode()) || arm_kD.hasChanged(hashCode()) || arm_kS.hasChanged(hashCode()) || arm_kV.hasChanged(hashCode()) || arm_kG.hasChanged(hashCode())) {
            armIO.setPID(arm_kP.get(), arm_kI.get(), arm_kD.get(), arm_kS.get(), arm_kV.get(), arm_kG.get());
        }
        if (arm_motionSpeed.hasChanged(hashCode()) || arm_motionAccel.hasChanged(hashCode()) || arm_motionJerk.hasChanged(hashCode())) {
            armIO.configureMotionMagic(arm_motionSpeed.get(), arm_motionAccel.get(), arm_motionJerk.get());
        }
    }

    // --- SubsystemChecker Implementation ---

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.addAll(armIO.getSelfCheckingHardware());
        hardware.addAll(indexer.getHardware());
        hardware.addAll(frontRollers.getHardware());

        for (SelfChecking device : hardware) {
            if (device.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) device.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        return armInputs.supplyCurrentAmps + indexer.getCurrent() + frontRollers.getCurrent();
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("Arm", armInputs.tempCelsius);
        temps.put("Rollers", indexer.getTemps().get("Indexer"));
        temps.put("FrontRollers", frontRollers.getTemps().get(IntakeConstants.frontRollersName));
        return temps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        // Split the limit or apply to both? Usually apply individually.
        armIO.setCurrentLimit(amps);
        frontRollers.setCurrentLimit(amps);
        indexer.setCurrentLimit(amps);
    }

    @Override
    protected Command systemCheckCommand() {
        return runOnce(() -> {
            // Simple check logic
            if (isFullyConnected()) {
                Logger.recordOutput("Intake" + "/SystemCheck/Connected", "GOOD");
            } else {
                Logger.recordOutput("Intake" + "/SystemCheck/Connected", "BAD");
            }
        }).withName("FuelIntakeSystemCheck");
    }
}