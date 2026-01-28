package frc.robot.utils.CompetitionFieldUtils.Simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Rebuilt2026FieldObjects.FuelOnFieldSimulated;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Rebuilt2026FieldObjects.RebuiltFuelOnFly;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RebuiltHub;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RebuiltOutpost;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;

//YEARLYUPDATE: change this field (field bounds/obstacles and scoring locations) to match the year's competition
/**
 * field simulation for 2025 competition
 * 
 */
public class Rebuilt2026FieldSimulation extends CompetitionFieldSimulation {
	private static final double FIELD_X_MIN = 0.00000;
	private static final double FIELD_X_MAX = 16.54105;
	private static final double FIELD_Y_MIN = 0.00000;
	private static final double FIELD_Y_MAX = 8.06926;

	private static final double HUB_X_LEN = 1.19380;
	private static final double HUB_Y_LEN = 1.19380;
	private static final double HUB_X = 4.625594;
	private static final double HUB_Y = 4.03463;
	private static final double HUB_RAMP_LENGTH = Inches.of(73.0).in(Meters);

	private static final double UPRIGHT_X_LEN = Inches.of(3.5).in(Meters);
	private static final double UPRIGHT_Y_LEN = Inches.of(1.5).in(Meters);
	private static final double UPRIGHT_OFFSET_FROM_END_WALL = 1.06204;
	private static final double UPRIGHT_OFFSET_FROM_SIDE_WALL = 3.31524;
	private static final double UPRIGHT_Y_SPACING = Inches.of(33.75).in(Meters);

	private static final double TRENCH_WALL_Y_LEN = Inches.of(12.0).in(Meters);
	private static final double TRENCH_WALL_X_LEN = Inches.of(47.0).in(Meters);
	private static final double TRENCH_WALL_OFFSET_FROM_END_WALL = 4.61769;
	private static final double TRENCH_WALL_OFFSET_FROM_SIDE_WALL = 1.43113;
 protected RebuiltHub blueHub;
    protected RebuiltHub redHub;

    protected RebuiltOutpost blueOutpost;
    protected RebuiltOutpost redOutpost;

    protected boolean isInEfficiencyMode = true;

    protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
    protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
    protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);
	public Rebuilt2026FieldSimulation(AbstractDriveTrainSimulation robot) {
		// Initialize faces
		super(robot, new RebuiltFieldObstaclesMap(true));
		blueHub = new RebuiltHub(this, true);
        super.addCustomSimulation(blueHub);

        redHub = new RebuiltHub(this, false);
        super.addCustomSimulation(redHub);

        blueOutpost = new RebuiltOutpost(this, true);
        super.addCustomSimulation(blueOutpost);

        redOutpost = new RebuiltOutpost(this, false);
        super.addCustomSimulation(redOutpost);
	}
public static double randomInRange(double variance) {
        return (Math.random() - 0.5) * variance;
    }
	public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double xVariance,
            double yVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        addGamePiece(new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
                new Translation2d(),
                new ChassisSpeeds(),
                yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance))));
    }
 @Override
    public void placeGamePiecesOnField(boolean preload) {
		//preload isn't allowed in 2026
        blueOutpost.reset();
        redOutpost.reset();

        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
                addGamePiece(new FuelOnFieldSimulated(centerPieceBottomRightCorner.plus(
                        new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
            }
        }

        boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
                && DriverStation.getAlliance().get() == Alliance.Blue;

        if (isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new FuelOnFieldSimulated(blueDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new FuelOnFieldSimulated(redDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
		}
    }
	
	/**
	 * the obstacles on the 2025 competition field
	 */
	public static final class RebuiltFieldObstaclesMap
			extends FieldObstaclesMap {
		public RebuiltFieldObstaclesMap(boolean AddRampCollider) {
			// blue wall
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MIN, FIELD_Y_MAX));

			// red wall
			addBorderLine(new Translation2d(FIELD_X_MAX, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));

			// scoring area walls
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MIN));

			// opposite side
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MAX), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));

			// blue tower uprights
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(UPRIGHT_OFFSET_FROM_END_WALL, UPRIGHT_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							UPRIGHT_OFFSET_FROM_END_WALL,
							UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
							new Rotation2d()));

			// red tower uprights
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
							new Rotation2d()));

			// blue trench wall
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(TRENCH_WALL_OFFSET_FROM_END_WALL, TRENCH_WALL_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							TRENCH_WALL_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));

			// red trench wall
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
							TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));

			// Colliders to describe the hub plus ramps
			if (AddRampCollider) {

				// blue hub + ramps
				addRectangularObstacle(
						HUB_X_LEN, HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));

				// red hub + ramps
				addRectangularObstacle(
						HUB_X_LEN,
						HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH,
						new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));

			} else {

				// blue hub
				addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));

				// red hub
				addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));
			}
		}

	}
}
