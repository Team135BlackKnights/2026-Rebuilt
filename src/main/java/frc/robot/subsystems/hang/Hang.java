package frc.robot.subsystems.hang;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.hang.climber.Climber;
import frc.robot.subsystems.hang.wedgeArm.WedgeArmIO;
import frc.robot.subsystems.hang.wedgeArm.WedgeArmIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class Hang extends SubsystemChecker {
    private final Climber climber;
    private final WedgeArmIO wedgeArmIO;
    private final WedgeArmIOInputsAutoLogged wedgeArmInputs = new WedgeArmIOInputsAutoLogged();

    public Hang(Climber climber, WedgeArmIO wedgeArmIO) {
        this.climber = climber;
        this.wedgeArmIO = wedgeArmIO;
    }

    @Override
    public void periodic() {
        wedgeArmIO.updateInputs(wedgeArmInputs);
        Logger.processInputs("Hang/WedgeArm", wedgeArmInputs);
        climber.periodic();
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

    @Override
    public double getCurrent() {
        return wedgeArmInputs.supplyCurrentAmps + climber.getCurrent();
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("Arm", wedgeArmInputs.tempCelsius);
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
            if (wedgeArmInputs.connected && climber.isConnected()) {
                Logger.recordOutput("Hang" + "/SystemCheck/WedgeConnected", "GOOD");
            } else {
                Logger.recordOutput("Hang" + "/SystemCheck/WedgeConnected", "BAD");
            }
        }).withName("HangSystemCheck");
    }
}
