package frc.robot.subsystems.hang;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.hang.climber.Climber;
import frc.robot.subsystems.hang.wedgeArm.WedgeArmIO;
import frc.robot.subsystems.hang.wedgeArm.WedgeArmIOInputsAutoLogged;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class Hang extends SubsystemChecker {
    private final Climber climber;
    private final WedgeArmIO wedgeArmIO;
    private final WedgeArmIOInputsAutoLogged wedgeArmInputs = new WedgeArmIOInputsAutoLogged();
    private final LoggableTunedNumber wedgeArmkP = new LoggableTunedNumber("Hang/WedgeArm/kP", 0.5,
            TuningConstants.isTuningClimber);
    private final LoggableTunedNumber wedgeArmkI = new LoggableTunedNumber("Hang/WedgeArm/kI", 0.0,
            TuningConstants.isTuningClimber);
    private final LoggableTunedNumber wedgeArmkD = new LoggableTunedNumber("Hang/WedgeArm/kD", 0.01,
            TuningConstants.isTuningClimber);
    private final LoggableTunedNumber wedgeArmkS = new LoggableTunedNumber("Hang/WedgeArm/kS", 0.0,
            TuningConstants.isTuningClimber);
    private final LoggableTunedNumber wedgeArmkV = new LoggableTunedNumber("Hang/WedgeArm/kV", 0.0,
            TuningConstants.isTuningClimber);
    private final LoggableTunedNumber wedgeArmSetpoint = new LoggableTunedNumber("Hang/WedgeArm/SetpointRads",
            Units.degreesToRadians(72), TuningConstants.isTuningClimber);

    public enum HangState {
        STOWED, EXTENDED, MOVING_UP, MOVING_DOWN
    }

    private HangState hangState = HangState.STOWED;

    public Hang(Climber climber, WedgeArmIO wedgeArmIO) {
        this.climber = climber;
        this.wedgeArmIO = wedgeArmIO;
        applyAllPIDs();
    }

    private void applyAllPIDs() {

        wedgeArmIO.setPID(
                wedgeArmkP.get(), wedgeArmkI.get(), wedgeArmkD.get(),
                wedgeArmkS.get(), wedgeArmkV.get());

    }

    private void updateTunablePIDs() {

        LoggableTunedNumber.ifChanged(
                hashCode(),
                () -> wedgeArmIO.setPID(
                        wedgeArmkP.get(), wedgeArmkI.get(), wedgeArmkD.get(),
                        wedgeArmkS.get(), wedgeArmkV.get()),
                wedgeArmkP, wedgeArmkI, wedgeArmkD, wedgeArmkS, wedgeArmkV);
    }

    @Override
    public void periodic() {
        wedgeArmIO.updateInputs(wedgeArmInputs);
        Logger.processInputs("Hang/WedgeArm", wedgeArmInputs);
        climber.periodic();
        updateTunablePIDs();
        // process state
        double setWedgeAngle = 0;
        Climber.Goal climberGoal = Climber.Goal.STOPPED;
        switch (hangState) {
            case STOWED:
                setWedgeAngle = 0;
                climberGoal = Climber.Goal.STOPPED;
                break;
            case EXTENDED:
                setWedgeAngle = wedgeArmSetpoint.get();
                climberGoal = Climber.Goal.STOPPED;
                break;
            case MOVING_UP:
                setWedgeAngle = wedgeArmSetpoint.get();
                // TODO: force lock?
                climberGoal = Climber.Goal.CLIMBING;
                break;
            case MOVING_DOWN:
                setWedgeAngle = wedgeArmSetpoint.get();
                // TODO: force lock?
                climberGoal = Climber.Goal.DROPPING;
                break;
        }
        wedgeArmIO.setPosition(setWedgeAngle);
        climber.setGoal(climberGoal);
    }
    public void setGoal(HangState state){
        this.hangState = state;
    }
    public double getAngle() {
        return wedgeArmInputs.positionRads;
    }

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.addAll(wedgeArmIO.getSelfCheckingHardware());
        hardware.addAll(climber.getHardware());

        for (SelfChecking device : hardware) {
            if (device.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) device.getHardware());
            }
        }
        return orchestra;
    }

    private boolean wedgeArmConnected() {
        return wedgeArmInputs.connected;
    }

    private boolean climberConnected() {
        return climber.isConnected();
    }

    @Override
    public double getCurrent() {
        return wedgeArmInputs.supplyCurrentAmps + climber.getCurrent();
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("WedgeArm", wedgeArmInputs.tempCelsius);
        temps.put("Climber", climber.getTemps().get(SimpleMechanismConstants.Climber.climberName));
        return temps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        // Split the limit or apply to both? Usually apply individually.
        wedgeArmIO.setCurrentLimit(amps);
        climber.setCurrentLimit(amps);
    }

    @Override
    protected Command systemCheckCommand() {
        return runOnce(() -> {
            // Simple check logic
            if (wedgeArmConnected() && climberConnected()) {
                Logger.recordOutput("Hang" + "/SystemCheck/WedgeConnected", "GOOD");
            } else {
                Logger.recordOutput("Hang" + "/SystemCheck/WedgeConnected", "BAD");
            }
        }).withName("HangSystemCheck");
    }
}
