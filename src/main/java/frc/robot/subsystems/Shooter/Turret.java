package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Shooter.azimuth.AzimuthIO;
import frc.robot.subsystems.Shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.azimuth.AzimuthIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.hood.HoodIOInputsAutoLogged;

import frc.robot.subsystems.Shooter.hood.HoodIO;
import frc.robot.utils.selfCheck.SelfChecking;

public class Turret extends SubsystemChecker {

    private final AzimuthIO azimuthIO;
    private final FlywheelIO flywheelIO;
    private final HoodIO hoodIO;
    private final AzimuthIOInputsAutoLogged azimuthInputs = new AzimuthIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    public Turret(AzimuthIO azimuthIO, FlywheelIO flywheelIO, HoodIO hoodIO) {
        this.azimuthIO = azimuthIO;
        this.flywheelIO = flywheelIO;
        this.hoodIO = hoodIO;
    }

  public HashMap<String, Double> getTemps() {
        HashMap<String, Double> tempMap = new HashMap<>();
        tempMap.put("Azimuth", azimuthInputs.tempCelsius);
        tempMap.put("Flywheel", flywheelInputs.tempCelsius);
        tempMap.put("Hood", hoodInputs.tempCelsius);
        return tempMap;
    }


    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.addAll(azimuthIO.getSelfCheckingHardware());
        hardware.addAll(flywheelIO.getSelfCheckingHardware());
        hardware.addAll(hoodIO.getSelfCheckingHardware());
        for (SelfChecking motor : hardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        return Math.abs(azimuthInputs.supplyCurrentAmps) + Math.abs(flywheelInputs.supplyCurrentAmps) + Math.abs(hoodInputs.supplyCurrentAmps);
    }
    @Override
    public void setCurrentLimit(int amps) {
        azimuthIO.setCurrentLimit(amps);
    }
    public void setCurrentLimit(int azimuthAmps, int flywheelAmps, int hoodAmps) {
        azimuthIO.setCurrentLimit(azimuthAmps);
        flywheelIO.setCurrentLimit(flywheelAmps);
        hoodIO.setCurrentLimit(hoodAmps);
    }

    @Override
    protected Command systemCheckCommand() {
        return runOnce(() -> {
            // TODO Auto-generated method stub

        });
    }
    
}
