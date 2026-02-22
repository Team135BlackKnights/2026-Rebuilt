package frc.robot.subsystems.Turret.azimuth.EasyCRT;
/**
 * Mechanism gearing for conversions from the motor output to the mechanism output.
 */
public class MechanismGearing
{

  /**
   * Mechanism gearbox attached to the motor.
   */
  private final GearBox            gearBox;

  /**
   * Construct a {@link MechanismGearing} with a reduction ratio.
   *
   * @param reductionRatio Reduction ratio. For example, a reduction of "3:1" is 3.0; a reduction of "1:2" is 0.5.
   */
  public MechanismGearing(double reductionRatio)
  {
    gearBox = GearBox.fromReductionStages(reductionRatio);
  }

  /**
   * Construct a {@link MechanismGearing} with a reduction ratios.
   *
   * @param reductionRatios Reduction ratio. For example, a reduction of "3:1" is 3.0; a reduction of "1:2" is 0.5.
   */
  public MechanismGearing(double... reductionRatios)
  {
    gearBox = GearBox.fromReductionStages(reductionRatios);
  }

  /**
   * Initialize the {@link MechanismGearing} with a {@link GearBox}
   *
   * @param gearBox   {@link GearBox} attached to the motor.
   */
  public MechanismGearing(GearBox gearBox)
  {
    this.gearBox = gearBox;
  }

  /**
   * Get the sensor to the mechanism ratio for the motor to the mechanism.
   *
   * @return OUT:IN or OUT/IN ratio to use for sensor to mechanism calculations.
   */
  public double getRotorToMechanismRatio()
  {
    double ratio = gearBox.getInputToOutputConversionFactor();
    return ratio;
  }

  /**
   * Get the mechanism rotation to sensor rotation ratio for the mechanism. AKA THE REDUCTION!
   *
   * @return IN:OUT or IN/OUT to use for mechanism to sensor calculations.
   */
  public double getMechanismToRotorRatio()
  {
    double ratio = gearBox.getOutputToInputConversionFactor();
    return ratio;
  }

  /**
   * Divide the gearbox reduction ratio by i.
   *
   * @param i Numerator.
   * @return {@link MechanismGearing}
   */
  public MechanismGearing div(double i)
  {
    gearBox.div(i);
    return this;
  }
}