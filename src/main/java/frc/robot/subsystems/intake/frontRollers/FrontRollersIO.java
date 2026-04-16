package frc.robot.subsystems.intake.frontRollers;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIO;

public interface FrontRollersIO extends GenericRollerSystemIO {
    default void setVelocity(double velocityRadsPerSec) {}
    default void setPID(double p, double d, double ks, double kv, double ka) {}
    default void setRamp(double closedLoopRampSecs) {}
    default boolean supportsVelocityControl() { return false; }
}
