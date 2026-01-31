package frc.robot.subsystems.arm;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.selfCheck.SelfChecking;

//TODO! Make intake subsystem which uses an arm *without* an encoder.
public abstract class Arm<G extends Arm.positionRadiansGoal> extends SubsystemChecker {
   
    public interface positionRadiansGoal {
    DoubleSupplier getPositionSupplier();
    }
    public abstract G getGoal();
    public ArmIO io;    
    public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    public Arm(ArmIO io){
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Lift", inputs);
        io.applyPID(getGoal().getPositionSupplier().getAsDouble());
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

    public double getCurrent() {
        return inputs.currentAmps;
    };

	public HashMap<String, Double> getTemps() {
        HashMap<String, Double> tempMap = new HashMap<>();
        tempMap.put(inputs.name, inputs.motorTemperatureC);
        return tempMap;
    };

	public void setCurrentLimit(int amps) {
        io.setCurrentLimit(amps);
    };
}
