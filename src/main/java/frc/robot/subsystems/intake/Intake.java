package frc.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;

public class Intake extends SubsystemChecker {

    // Tuning 
    private static final LoggableTunedNumber arm_kP = new LoggableTunedNumber("Intake/Arm/kP", 5.0);
    private static final LoggableTunedNumber arm_kI = new LoggableTunedNumber("Intake/Arm/kI", 0.0);
    private static final LoggableTunedNumber arm_kD = new LoggableTunedNumber("Intake/Arm/kD", 0.1);
    private static final LoggableTunedNumber arm_kS = new LoggableTunedNumber("Intake/Arm/kS", 0.0);
    private static final LoggableTunedNumber arm_kV = new LoggableTunedNumber("Intake/Arm/kV", 0.0);
    
    // Setpoints
    private static final LoggableTunedNumber angle_stow = new LoggableTunedNumber("Intake/Setpoints/StowRads", Math.PI / 2.0);
    private static final LoggableTunedNumber angle_ground = new LoggableTunedNumber("Intake/Setpoints/GroundRads", 0.0);
    private static final LoggableTunedNumber angle_score = new LoggableTunedNumber("Intake/Setpoints/ScoreRads", Math.PI / 4.0);
    private static final LoggableTunedNumber roller_volts_intake = new LoggableTunedNumber("Intake/Rollers/IntakeVolts", 8.0);
    private static final LoggableTunedNumber roller_volts_hold = new LoggableTunedNumber("Intake/Rollers/HoldVolts", 0.5);
    private static final LoggableTunedNumber roller_volts_eject = new LoggableTunedNumber("Intake/Rollers/EjectVolts", -10.0);

    //Tolerance
    private static final LoggableTunedNumber arm_tolerance = new LoggableTunedNumber("Intake/ToleranceRads", 0.05);

    // IO  
    private final ArmIO armIO;
    private final RollerIO rollerIO;

    //Inputs
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

    // State Machin
    public enum Goal {
        STOW, // Arm up, rollers slow/stop
        INTAKE_GROUND, // Arm down, rollers intake
        SCORE // Arm to score angle, rollers eject
    }

    private Goal goal = Goal.STOW;
    private double currentArmSetpoint = 0.0;
    private double currentRollerVolts = 0.0;

    public Intake(ArmIO armIO, RollerIO rollerIO) {
        this.armIO = armIO;
        this.rollerIO = rollerIO;
        
        // Apply initial PID
        updateTunablePIDs();
    }

    @Override
    public void periodic() {
        // 1. Update Inputs
        armIO.updateInputs(armInputs);
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Intake/Arm", armInputs);
        Logger.processInputs("Intake/Rollers", rollerInputs);

        // 2. Check for Tuning Updates
        updateTunablePIDs();

        // 3. Safety Check
        if (DriverStation.isDisabled()) {
            goal = Goal.STOW; // Reset state on disable
            armIO.stop();
            rollerIO.stop();
            return;
        }

        // 4. State Machine Logic
        switch (goal) {
            case STOW -> {
                currentArmSetpoint = angle_stow.get();
                // Apply a small holding voltage if we have game piece, or 0 if empty (logic simplified here)
                currentRollerVolts = roller_volts_hold.get(); 
            }
            case INTAKE_GROUND -> {
                currentArmSetpoint = angle_ground.get();
                currentRollerVolts = roller_volts_intake.get();
            }
            case SCORE -> {
                currentArmSetpoint = angle_score.get();
                // Only eject if we are close to the angle? Or just eject?
                // Typically you wait for the arm to be ready:
                if (Math.abs(armInputs.positionRads - currentArmSetpoint) < arm_tolerance.get()) {
                    currentRollerVolts = roller_volts_eject.get();
                } else {
                    currentRollerVolts = 0.0; // Wait to arrive
                }
            }
        }

        // 5. Apply Outputs
        armIO.setPosition(currentArmSetpoint);
        rollerIO.setVoltage(currentRollerVolts);

        // 6. Logging
        Logger.recordOutput("Intake/Goal", goal);
        Logger.recordOutput("Intake/SetpointAngle", currentArmSetpoint);
        Logger.recordOutput("Intake/SetpointVolts", currentRollerVolts);
        Logger.recordOutput("ntake/AtSetpoint", isAtSetpoint());
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public boolean isAtSetpoint() {
        return Math.abs(armInputs.positionRads - currentArmSetpoint) < arm_tolerance.get();
    }

    private void updateTunablePIDs() {
        if (arm_kP.hasChanged(hashCode()) || arm_kI.hasChanged(hashCode()) || arm_kD.hasChanged(hashCode())) {
            armIO.setPID(arm_kP.get(), arm_kI.get(), arm_kD.get(), arm_kS.get(), arm_kV.get());
        }
    }

    // --- SubsystemChecker Implementation ---

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.addAll(armIO.getSelfCheckingHardware());
        hardware.addAll(rollerIO.getSelfCheckingHardware());

        for (SelfChecking device : hardware) {
            if (device.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) device.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        return armInputs.supplyCurrentAmps + rollerInputs.supplyCurrentAmps;
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("Arm", armInputs.tempCelsius);
        temps.put("Rollers", rollerInputs.tempCelsius);
        return temps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        // Split the limit or apply to both? Usually apply individually.
        armIO.setCurrentLimit(amps);
        rollerIO.setCurrentLimit(amps);
    }

    @Override
    protected Command systemCheckCommand() {
        return runOnce(() -> {
            // Simple check logic
             if (armInputs.connected && rollerInputs.connected) {
                 // Good
             }
        }).withName("FuelIntakeSystemCheck");
    }
}