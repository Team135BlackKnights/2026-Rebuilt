package frc.robot.subsystems.arm;


import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public abstract class ArmIOKrakenFOC implements ArmIO {
  private final TalonFX talon;
  private TalonFXConfiguration config;
  private final String name;
  private final double reduction;
  private final PIDController pid;

  public ArmIOKrakenFOC(int id, String name, double reduction,
      int currentLimitAmps, double kP, double kI,
      double kD, boolean brake) {
    this.reduction = reduction;

    this.talon = new TalonFX(id);


    this.name = name;

    this.pid = new PIDController(kP, kI, kD);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast; 
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  public void updateInputs(LiftIOInputs inputs) {
    //TODO: add correct update inputs
  }

  public void setAngle(double angleRads) {
    //TODO: convert angle to voltage using pid
  }

  @Override
  public List<SelfChecking> getSelfCheckingHardware() {
    List<SelfChecking> hardware = new ArrayList<SelfChecking>();
    hardware.add(new SelfCheckingTalonFX(name, talon));

    return hardware;
  }

  @Override
  public void setCurrentLimit(int amps) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = amps;
    talon.getConfigurator().apply(config);
  }
}