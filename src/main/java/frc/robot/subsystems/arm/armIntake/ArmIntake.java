package frc.robot.subsystems.arm.armIntake;

import java.util.function.DoubleSupplier;

import org.checkerframework.checker.units.qual.t;

import com.ctre.phoenix6.HootSchemaType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class ArmIntake extends Arm<ArmIntake.Goal> {
    public enum Goal implements Arm.positionRadiansGoal{
        GROUND(()->0),
        FOLDED(()->3*Math.PI/4); //change to loggabletunednumber

    private final DoubleSupplier positionSupplier;
    
    Goal(LoggableTunedNumber positionSupplier){
        this.positionSupplier = positionSupplier::get;
    }

    Goal(DoubleSupplier positionSupplier){
        this.positionSupplier = positionSupplier;
    }
    
    public DoubleSupplier getPositionSupplier(){
        return positionSupplier;
    }
    }
    public ArmIntake(ArmIntakeIO io){
        super(io);
    }

    private static Goal goal = Goal.GROUND;

    public Goal getGoal(){
        return goal;
    }

    protected Command systemCheckCommand(){
        return null;

    }
    public static void setGoal(ArmIntake.Goal desGoal){
        goal = desGoal;
    }
}