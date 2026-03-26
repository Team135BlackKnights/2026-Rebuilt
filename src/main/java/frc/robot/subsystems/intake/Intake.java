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
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;
import lombok.Getter;

public class Intake extends SubsystemChecker {

    private static final LoggableTunedNumber arm_kP = new LoggableTunedNumber("Intake/Arm/kP", 2,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kI = new LoggableTunedNumber("Intake/Arm/kI", 0.0,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kD = new LoggableTunedNumber("Intake/Arm/kD", 0.1,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kS = new LoggableTunedNumber("Intake/Arm/kS", 4.1,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kV = new LoggableTunedNumber("Intake/Arm/kV", 0,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_kG = new LoggableTunedNumber("Intake/Arm/kG", 0,
            TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_motionSpeed = new LoggableTunedNumber(
            "Intake/Arm/MotionCruiseInchesPerSec", 200, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_motionAccel = new LoggableTunedNumber(
            "Intake/Arm/MotionAccelInchesPerSec2", 50, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber arm_neutralBand = new LoggableTunedNumber("Intake/Arm/NeutralBand", .15,
            TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber slide_stow = new LoggableTunedNumber("Intake/Setpoints/StowInches",
            IntakeConstants.slideMinInches+1, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber slide_ground = new LoggableTunedNumber("Intake/Setpoints/GroundInches",
            10.875, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber time_jackhammer = new LoggableTunedNumber("Intake/JackhammerTimeSecs",
            .25, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber slide_agitate = new LoggableTunedNumber("Intake/Setpoints/AgitateInches",
            6.0, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber agitate_force_volts = new LoggableTunedNumber(
            "Intake/AgitateForceVolts", -5.5, TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber arm_tolerance = new LoggableTunedNumber("Intake/ToleranceInches", 0.5,
            TuningConstants.isTuningIntake);

    private final ArmIO armIO;
    private final FrontRollers frontRollers;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    protected Goal lastGoal;
    private final Timer stateTimer = new Timer();

    public enum Goal {
        START,
        STOW,
        INTAKE_GROUND,
        INTAKE_GROUND_SHOOT,
        INTAKE_OUTER_IDLE,
        JACKHAMMERING_OUT,
        JACKHAMMERING_IN,
        VOMITING,
        SHOOTING,
        HOLD,
        AGITATING,
        TUNING,
    }

    private Goal goal = Goal.START;
    private double currentSlideSetpointInches = 0.0;
    private double currentRollerVolts = 0.0;
    private boolean agitatingGoingUp = true;
    @Getter
    private boolean intakeDeployed = false;

    public Intake(ArmIO armIO, FrontRollers frontRollers) {
        this.armIO = armIO;
        this.frontRollers = frontRollers;
        updateTunablePIDs();
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Intake/Arm", armInputs);
        frontRollers.periodic();

        updateTunablePIDs();

        if (DriverStation.isDisabled()) {
            goal = Goal.INTAKE_OUTER_IDLE;
            armIO.stop();
            frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            return;
        }

        if (getGoal() != lastGoal) {
            stateTimer.reset();
            lastGoal = getGoal();
        }

        switch (goal) {
            case TUNING -> frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            case START -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            }
            case STOW -> {
                currentSlideSetpointInches = slide_stow.get();
                frontRollers.setGoal(FrontRollers.Goal.IDLING);
            }
            case INTAKE_GROUND -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(FrontRollers.Goal.INTAKING);
            }
            case INTAKE_GROUND_SHOOT -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(FrontRollers.Goal.SHOOTING);
            }
            case INTAKE_OUTER_IDLE -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(FrontRollers.Goal.IDLING);
            }
            case VOMITING -> {
                currentSlideSetpointInches = slide_stow.get();
                frontRollers.setGoal(FrontRollers.Goal.VOMITING);
            }
            case JACKHAMMERING_IN -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(getJackhammerFrontRollerGoal());
            }
            case JACKHAMMERING_OUT -> {
                currentSlideSetpointInches = slide_ground.get();
                frontRollers.setGoal(getJackhammerFrontRollerGoal());
            }
            case SHOOTING -> frontRollers.setGoal(FrontRollers.Goal.SHOOTING);
            case HOLD -> frontRollers.setGoal(FrontRollers.Goal.STOPPED);
            case AGITATING -> {
                double agitateInches = slide_agitate.get();
                double groundInches = slide_ground.get();
                if (agitatingGoingUp) {
                    currentSlideSetpointInches = agitateInches;
                    if (armInputs.positionInches <= agitateInches) {
                        agitatingGoingUp = false;
                        currentSlideSetpointInches = groundInches;
                    }
                } else {
                    currentSlideSetpointInches = groundInches;
                    if (Math.abs(armInputs.positionInches - groundInches) < arm_tolerance.get()) {
                        agitatingGoingUp = true;
                    }
                }
                frontRollers.setGoal(FrontRollers.Goal.INTAKING);
            }
        }

        if (goal != Goal.TUNING) {
            if (goal == Goal.AGITATING && agitatingGoingUp
                    && armInputs.positionInches > currentSlideSetpointInches) {
                armIO.setVoltage(agitate_force_volts.get());
            } else {
                armIO.setPosition(currentSlideSetpointInches);
            }
        }

        Logger.recordOutput("Intake/Goal", goal);
        Logger.recordOutput("SuperStructure/Intake/IntakeGoal", goal);
        Logger.recordOutput("Intake/SetpointInches", currentSlideSetpointInches);
        Logger.recordOutput("Intake/SetpointVolts", currentRollerVolts);
        Logger.recordOutput("Intake/AtSetpoint", isAtSetpoint());
        Logger.recordOutput("Intake/AgitatingGoingUp", agitatingGoingUp);
        Logger.recordOutput("Intake/AgitateForceInActive",
                goal == Goal.AGITATING && agitatingGoingUp && armInputs.positionInches > currentSlideSetpointInches);
    }

    private FrontRollers.Goal getJackhammerFrontRollerGoal() {
        double t = Timer.getFPGATimestamp() % (time_jackhammer.get() * 2.0);
        return t < time_jackhammer.get() ? FrontRollers.Goal.JACKHAMMER_IN : FrontRollers.Goal.JACKHAMMER_OUT;
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
        if (goal == Goal.AGITATING) {
            agitatingGoingUp = true;
        }
        if (goal == Goal.INTAKE_GROUND || goal == Goal.INTAKE_OUTER_IDLE || goal == Goal.AGITATING) {
            intakeDeployed = true;
        } else if (goal == Goal.STOW) {
            intakeDeployed = false;
        }
    }

    public void zero() {
        armIO.zero();
    }

    public Goal getGoal() {
        return goal;
    }

    public void runCharacterization(double volts) {
        goal = Goal.TUNING;
        armIO.setVoltage(volts);
    }

    public void holdAtCurrentPosition() {
        currentSlideSetpointInches = armInputs.positionInches;
        setGoal(Goal.HOLD);
    }

    public double getCharacterizationMeasurement() {
        return armInputs.positionInches;
    }

    public double getCharVelocity() {
        return armInputs.velocityInchesPerSec;
    }

    public boolean isAtSetpoint() {
        return Math.abs(armInputs.positionInches - currentSlideSetpointInches) < arm_tolerance.get();
    }

    public double getSlidePositionInches() {
        return armInputs.positionInches;
    }

    private boolean isArmConnected() {
        return armInputs.connected;
    }

    private boolean isFrontRollersConnected() {
        return frontRollers.isConnected();
    }

    public boolean isFullyConnected() {
        return isArmConnected() && isFrontRollersConnected();
    }

    private void updateTunablePIDs() {
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            armIO.setPID(
                    arm_kP.get(), arm_kI.get(), arm_kD.get(),
                    arm_kS.get(), arm_kV.get(), arm_kG.get(),
                    arm_motionSpeed.get(), arm_motionAccel.get(), arm_neutralBand.get());
        }, arm_kP, arm_kI, arm_kD, arm_kS, arm_kV, arm_kG,
                arm_motionSpeed, arm_motionAccel, arm_neutralBand);
    }

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.addAll(armIO.getSelfCheckingHardware());
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
        return armInputs.supplyCurrentAmps + frontRollers.getCurrent();
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("Arm", armInputs.tempCelsius);
        temps.put("FrontRollers", frontRollers.getTemps().get(IntakeConstants.frontRollersName));
        return temps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        armIO.setCurrentLimit(amps);
        frontRollers.setCurrentLimit(amps);
    }

    @Override
    protected Command systemCheckCommand() {
        return runOnce(() -> {
            if (isFullyConnected()) {
                Logger.recordOutput("Intake" + "/SystemCheck/Connected", "GOOD");
            } else {
                Logger.recordOutput("Intake" + "/SystemCheck/Connected", "BAD");
            }
        }).withName("FuelIntakeSystemCheck");
    }
}
