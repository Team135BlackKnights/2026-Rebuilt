// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OrchestraC;
import frc.robot.commands.RoughPIDCharacterization;
import frc.robot.commands.StaticCharacterization;
import frc.robot.commands.auto.AutoIntake;
import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Turret.ShotCalculator;
import frc.robot.subsystems.Turret.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.azimuth.AzimuthIO;
import frc.robot.subsystems.Turret.azimuth.AzimuthIOKrakenFOC;
import frc.robot.subsystems.Turret.azimuth.AzimuthIOSim;
import frc.robot.subsystems.Turret.flywheel.FlywheelIO;
import frc.robot.subsystems.Turret.flywheel.FlywheelIOKrakenFOC;
import frc.robot.subsystems.Turret.flywheel.FlywheelIOSim;
import frc.robot.subsystems.Turret.hood.HoodIO;
import frc.robot.subsystems.Turret.hood.HoodIOKrakenFOC;
import frc.robot.subsystems.Turret.hood.HoodIOSim;
import frc.robot.subsystems.Turret.kickup.Kickup;
import frc.robot.subsystems.Turret.kickup.KickupIO;
import frc.robot.subsystems.Turret.kickup.KickupIOSim;
import frc.robot.subsystems.Turret.kickup.KickupIOSparkBase;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import frc.robot.subsystems.drive.Mecanum.MecanumIO;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSparkBase;
import frc.robot.subsystems.drive.Mecanum.MecanumIOTalonFX;
import frc.robot.subsystems.drive.FastSwerve.ModuleIO;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOCShifting;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOCWithThrifty;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Goal;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.intake.arm.ArmIOSim;
import frc.robot.subsystems.intake.frontRollers.FrontRollers;
import frc.robot.subsystems.intake.frontRollers.FrontRollersIO;
import frc.robot.subsystems.intake.frontRollers.FrontRollersIOKrakenFOC;
import frc.robot.subsystems.intake.frontRollers.FrontRollersIOSim;
import frc.robot.subsystems.intake.indexer.Indexer;
import frc.robot.subsystems.intake.indexer.IndexerIO;
import frc.robot.subsystems.intake.indexer.IndexerIOKrakenFOC;
import frc.robot.subsystems.intake.indexer.IndexerIOSim;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.utils.CompetitionFieldUtils.Simulation.AIRobotInSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.MecanumDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.TankDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.GyroSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOSouthmoon;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.AprilTagLayoutType;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;
import frc.robot.utils.drive.Sensors.GyroIOSim;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TuningConstants;

import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;

import frc.robot.utils.GeomUtil;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Rebuilt2026FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.robotToggles.Toggles;
import frc.robot.utils.robotToggles.TogglesIO;
import frc.robot.utils.robotToggles.TogglesIOHardware;
import frc.robot.utils.robotToggles.TogglesIONetworkTables;

import frc.robot.utils.Touchboard.PosePlotterUtil;
import frc.robot.utils.Touchboard.TouchboardAutoFactory;
import frc.robot.utils.Touchboard.JukeboxUtil;
import frc.robot.utils.Touchboard.TouchboardAutoPlan;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

/**
 * This code depends on WPILib 2025, Choreo 2025, PhotonLib 2025, Studica,
 * Phoenix-6 2025 (non-replay), REVLib 2025, URCL, GrappleLib 2025, AKit 2025,
 * and PathplannerLib 2025.
 * IT WILL NOT WORK WITHOUT ANY OF THESE!
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	// private static final LEDs leds = new LEDs();
	public static Vision visionS;
	public static Toggles toggles;
	public static LocalADStarAK pathFinder = new LocalADStarAK();
	public static TouchboardAutoFactory touchboardAutoFactory;
	// public static Hang hang;
	public static Kickup kickup;
	public static Turret leftTurret;
	public static Turret rightTurret;
	public static Intake intake;
	private final LoggedDashboardChooser<Command> autoChooser;
	// [Map<String,>,]
	public static CommandXboxController driveController = new CommandXboxController(0);
	public static CommandXboxController manipController = new CommandXboxController(1);
	// public static DriverStationHID dsHIDHandler = new DriverStationHID(2);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	/** Timer used to force a shot after 0.25 s even if setpoints aren't reached */
	private final Timer shootTimer = new Timer();
	// Auto-jackhammer during shooting: shoot for this long, then jackhammer for the rest of the cycle
	private static final LoggableTunedNumber shootCycleShootSec = new LoggableTunedNumber(
			"Shooting/CycleShootSec", 1.5, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber shootCycleJackhammerSec = new LoggableTunedNumber(
			"Shooting/CycleJackhammerSec", 0.5, TuningConstants.isTuningMacros);
	public static double angularSpeed = 0;
	public static double xSpeed = 0;
	public static double ySpeed = 0;
	Trigger aButtonDrive = driveController.a();
	Trigger bButtonDrive = driveController.b();
	Trigger xButtonDrive = driveController.x();
	Trigger yButtonDrive = driveController.y();
	Trigger leftBumperDrive = driveController.leftBumper();
	Trigger rightBumperDrive = driveController.rightBumper();
	Trigger leftTriggerDrive = driveController.leftTrigger(.125); // partial
	Trigger leftTriggerDriveFull = driveController.leftTrigger(.875); // full if needed
	Trigger rightTriggerDrive = driveController.rightTrigger(.125);
	Trigger rightTriggerDriveFull = driveController.rightTrigger(.875);
	Trigger leftStickButtonDrive = driveController.leftStick();
	Trigger rightStickButtonDrive = driveController.rightStick();
	Trigger selectButtonDrive = driveController.back(); // select
	Trigger startButtonDrive = driveController.start();
	Trigger manipRightTrigger = manipController.rightTrigger(.1);
	Trigger manipLeftTrigger = manipController.leftTrigger(.1);
	Trigger manipAButton = manipController.a();
	Trigger manipXButton = manipController.x();
	Trigger manipUpPov = manipController.pov(0);
	Trigger manipDownPov = manipController.pov(180);
	Trigger manipBButton = manipController.b();
	Trigger manipYButton = manipController.y();
	public static int currentTest = 0;
	public static String piConnection = "DISCONNECTED";
	@AutoLogOutput(key = "RobotState/currentPath")
	public static String currentPath = "";
	public static Field2d field = new Field2d();
	public static boolean userDrive = true;
	public static boolean withinLineTolerance = false;
	// Simulation
	public static Rebuilt2026FieldSimulation fieldSimulation = null;
	public static Command currentAuto, lastAuto = null;
	public static Map<String, Pair<Pose2d, Pose2d>> autoPaths = new HashMap<>();
	public static String closestChoreoPath = ""; // auto updates from Pathfinder, DON'T TOUCH!
	public static boolean grabbingAlgae = false; // auto updates from Pathfinder, DON'T TOUCH!
	int currentUpdate = 0;
	public static Pose2d startingPoseCache = new Pose2d();
	// Adjustable PathFollowing
	public static LoggableTunedNumber pathFollowingMaxLinearSpeed = new LoggableTunedNumber(
			"PathFollowing/MaxLinearSpeed", 5.5,
			TuningConstants.isTuningMacros),
			pathFollowingMaxAngularSpeed = new LoggableTunedNumber("PathFollowing/MaxAngularSpeed",
					DriveConstants.pathConstraints.maxAngularVelocityRadPerSec(), TuningConstants.isTuningMacros),
			pathFollowingMaxAcceleration = new LoggableTunedNumber("PathFollowing/MaxAcceleration", 30,
					TuningConstants.isTuningMacros),
			pathFollowingMaxAngularAcceleration = new LoggableTunedNumber("PathFollowing/MaxAngularAcceleration",
					DriveConstants.pathConstraints.maxAngularAccelerationRadPerSecSq(), TuningConstants.isTuningMacros),
			pathFollowingToleranceBeforeCustom = new LoggableTunedNumber("PathFollowing/ToleranceBeforeCustom",
					1,
					TuningConstants.isTuningMacros),
			pathFollowingToleranceDuringExactLineUp = new LoggableTunedNumber(
					"PathFollowing/ToleranceDuringExactLineup", .045,
					TuningConstants.isTuningMacros),
			scoreMinimumSpeed = new LoggableTunedNumber("PathFollowing/ScoreMaximumSpeed", .1,
					TuningConstants.isTuningMacros),
			leaveMinimumSpeed = new LoggableTunedNumber("PathFollowing/LeaveMinimumSpeed", .5,
					TuningConstants.isTuningMacros);
	ModuleLimits normalSpeeds = DriveConstants.moduleLimitsLow;

	public static void precalculateAllStartAndEndChoreos() {
		File choreoDirectory = new File(Filesystem.getDeployDirectory(),
				"choreo/");
		for (String choreo : choreoDirectory.list()) {
			// count number of . in the name using regex
			int dotCount = choreo.split("\\.", -1).length - 1;
			if (choreo.contains(".traj") && dotCount == 1) {
				// remove the .traj from the name
				choreo = choreo.replace(".traj", "");
				try {
					// get only the first path, as all choreo files should only have one path
					PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
					autoPaths.put(choreo, new Pair<>(path.getStartingHolonomicPose().get(), new Pose2d(
							path.getPoint(path.getAllPathPoints().size() - 1).position,
							path.getGoalEndState().rotation())));
					// put the flipped/mirrored path in the map as well
					autoPaths.put(choreo + "Flipped",
							new Pair<>(GeomUtil.apply(path.getStartingHolonomicPose().get(), true),
									GeomUtil.apply(new Pose2d(
											path.getPoint(path.getAllPathPoints().size() - 1).position,
											path.getGoalEndState().rotation().plus(new Rotation2d(Math.PI))), true)));
				} catch (FileVersionException | IOException | org.json.simple.parser.ParseException
						| NullPointerException e) {
					e.printStackTrace();
				}

			}
		}
	}

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands. y * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		/*
		 * These example states were originally used in 2025, retrofit to be an example
		 * (placeholder levels used)
		 * essentially each one of these choosers corresponds to a macro segment which
		 * then returns a series of commands
		 */
		DriverStation.silenceJoystickConnectionWarning(true);
		// We check to see what drivetrain type we have here, and create the correct
		// drivetrain system based on that.
		// If we get something wacky, throw an error
		String raw = PosePlotterUtil.getAutoString();
		Pose2d startingPose = new Pose2d();

		if (raw != null) {
			var planOpt = PosePlotterUtil.tryGetPlan();
			if (planOpt.isPresent()) {
				TouchboardAutoPlan plan = planOpt.get();

				// startPose from JSON (fallback if missing)
				if (plan.startPose != null) {
					startingPose = new Pose2d(
							plan.startPose.x,
							GeomUtil.applyY(plan.startPose.y, true),
							new Rotation2d());
				} else {
					startingPose = new Pose2d(4.398, 7.586, new Rotation2d());
				}
			} else {
				startingPose = new Pose2d(4.398, 7.586, new Rotation2d());
			}
		} else {
			startingPose = new Pose2d(4.398, 7.586, new Rotation2d());
		}
		startingPose = GeomUtil.apply(startingPose, false);
		switch (Constants.currentMode) {
			case REAL:
				switch (DriveConstants.driveType) {
					case SWERVE:

						switch (DriveConstants.robotMotorController) {
							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:
								switch (DriveConstants.gyroType) {
									case NAVX:
										switch (DriveConstants.swerveModuleType) {
											case SHIFTING_THIFTYSWERVE:
												// ignore encoder type, assume cancoder
												drivetrainS = new Swerve(new GyroIONavX(),
														new ModuleIOKrakenFOCShifting(0),
														new ModuleIOKrakenFOCShifting(1),
														new ModuleIOKrakenFOCShifting(2),
														new ModuleIOKrakenFOCShifting(3));
												break;
											case THRIFTYSWERVE:
											case SDSMK4I:
												if (DriveConstants.useThriftyEncoder) {
													drivetrainS = new Swerve(new GyroIONavX(),
															new ModuleIOKrakenFOCWithThrifty(0),
															new ModuleIOKrakenFOCWithThrifty(1),
															new ModuleIOKrakenFOCWithThrifty(2),
															new ModuleIOKrakenFOCWithThrifty(3));
												} else {
													drivetrainS = new Swerve(new GyroIONavX(),
															new ModuleIOKrakenFOC(0),
															new ModuleIOKrakenFOC(1),
															new ModuleIOKrakenFOC(2),
															new ModuleIOKrakenFOC(3));
												}
												break;
										}
										break;
									case PIGEON:
										switch (DriveConstants.swerveModuleType) {
											case SHIFTING_THIFTYSWERVE:
												drivetrainS = new Swerve(new GyroIOPigeon2(),
														new ModuleIOKrakenFOCShifting(0),
														new ModuleIOKrakenFOCShifting(1),
														new ModuleIOKrakenFOCShifting(2),
														new ModuleIOKrakenFOCShifting(3));
												break;
											case THRIFTYSWERVE:
											case SDSMK4I:
												drivetrainS = new Swerve(new GyroIOPigeon2(),
														new ModuleIOKrakenFOC(0),
														new ModuleIOKrakenFOC(1),
														new ModuleIOKrakenFOC(2),
														new ModuleIOKrakenFOC(3));
												break;
										}
										break;
									default:
										break;
								}
								break;
							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:
								switch (DriveConstants.gyroType) {
									case NAVX:
										drivetrainS = new Swerve(new GyroIONavX(),
												new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
												new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
										break;
									case PIGEON:
										drivetrainS = new Swerve(new GyroIOPigeon2(),
												new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
												new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
									default:
										break;
								}
								break;
						}
						break;
					case TANK:
						switch (DriveConstants.robotMotorController) {
							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:

								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Tank(
												new TankIOTalonFX(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Tank(new TankIOTalonFX(new GyroIONavX()));
										break;
								}
								break;
							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:

								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Tank(
												new TankIOSparkBase(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Tank(new TankIOSparkBase(new GyroIONavX()));
										break;
								}
								break;
						}
						break;
					case MECANUM:
						switch (DriveConstants.robotMotorController) {

							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Mecanum(
												new MecanumIOTalonFX(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Mecanum(
												new MecanumIOTalonFX(new GyroIONavX()));
										break;
								}
								break;

							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Mecanum(
												new MecanumIOSparkBase(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Mecanum(
												new MecanumIOSparkBase(new GyroIONavX()));
										break;
								}
								break;
						}
						break;
					// Placeholder values
					default:
						throw new IllegalArgumentException(
								"Unknown drivetrain implementation type, please check DriveConstants.java!");
				}
				visionS = new Vision(() -> getSelectedAprilTagLayout(),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "IntakeCam", 0,
								VisionConstants.cameras[0]),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackRightCam", 1,
								VisionConstants.cameras[1]),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackLeftCam", 2,
								VisionConstants.cameras[2]));
				/**
				 * visionS = new Vision(() -> getSelectedAprilTagLayout(),
				 * new VisionIOPhotonVision(() -> getSelectedAprilTagLayout(),
				 * VisionConstants.cameras[0].getId(),
				 * GeomUtil.poseToTransform3d(VisionConstants.cameras[0].getPose().get())),
				 * new VisionIOPhotonVision(() -> getSelectedAprilTagLayout(),
				 * VisionConstants.cameras[1].getId(),
				 * GeomUtil.poseToTransform3d(VisionConstants.cameras[1].getPose().get())),
				 * new VisionIOPhotonVision(() -> getSelectedAprilTagLayout(),
				 * VisionConstants.cameras[2].getId(),
				 * GeomUtil.poseToTransform3d(VisionConstants.cameras[2].getPose().get())),
				 * new VisionIOPhotonVision(() -> getSelectedAprilTagLayout(),
				 * VisionConstants.cameras[3].getId(),
				 * GeomUtil.poseToTransform3d(VisionConstants.cameras[3].getPose().get())));
				 */
				// Advanced Mechs Require Toggles
				toggles = new Toggles(new TogglesIOHardware());
				System.out.println("REAL SETUP DONE!");
				/*
				 * switch (SimpleMechanismConstants.Climber.climbMotorType) {
				 * case CTRE_ON_RIO:
				 * case CTRE_ON_CANIVORE:
				 * hang = new Hang(new Climber(new
				 * ClimberIOKrakenFOC(SimpleMechanismConstants.Climber.climberId,
				 * SimpleMechanismConstants.Climber.bus,
				 * SimpleMechanismConstants.Climber.climberName,
				 * SimpleMechanismConstants.Climber.climbCurrentLimit,
				 * SimpleMechanismConstants.Climber.climbInverted, true,
				 * SimpleMechanismConstants.Climber.climbReductionToClimbRollers)),
				 * new WedgeArmIOKrakenFOC(SimpleMechanismConstants.Climber.bus,
				 * SimpleMechanismConstants.Climber.wedgeArmId,
				 * SimpleMechanismConstants.Climber.wedgeArmServoId,
				 * SimpleMechanismConstants.Climber.wedgeArmName,
				 * SimpleMechanismConstants.Climber.wedgeArmCurrentLimit,
				 * SimpleMechanismConstants.Climber.wedgeArmInverted, true,
				 * SimpleMechanismConstants.Climber.wedgeReduction));
				 * break;
				 * default:
				 * throw new IllegalArgumentException(
				 * "Unknown implementation type for climber (REV NOT SUPPORTED!), please check SimpleMechanismConstants.java!"
				 * );
				 * }
				 */
				// Intake
				ArmIO armIO = new ArmIOKrakenFOC(Robot.rioCanBus,
						IntakeConstants.intakeMotorID,
						"IntakeArm",
						IntakeConstants.intakeCurrentLimit,
						IntakeConstants.intakeInverted,
						true,
						IntakeConstants.intakeArmReduction);
				Indexer indexer = new Indexer(
						new IndexerIOKrakenFOC(
								IntakeConstants.indexerMotorID,
								Robot.rioCanBus,
								"Indexer",
								IntakeConstants.indexerCurrentLimit,
								IntakeConstants.indexerInverted,
								false,
								IntakeConstants.intakeReductionToIndexerRollers));
				FrontRollers frontRollers = new FrontRollers(
						new FrontRollersIOKrakenFOC(IntakeConstants.frontRollersMotorID, Robot.rioCanBus,
								IntakeConstants.frontRollersName, IntakeConstants.frontRollersCurrentLimit,
								IntakeConstants.frontRollersInverted, false,
								IntakeConstants.frontRollersReduction));
				intake = new Intake(armIO, indexer, frontRollers);
				// Left Turret
				AzimuthIO azimuthIOLeftTurret = new AzimuthIOKrakenFOC(Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftAzimuthID,
						AdvancedMechanismConstants.Turret.leftAzimuthBigEncoderID,
						AdvancedMechanismConstants.Turret.leftAzimuthSmallEncoderID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitAzimuth,
						AdvancedMechanismConstants.Turret.minTurretAngle,
						AdvancedMechanismConstants.Turret.maxTurretAngle,
						AdvancedMechanismConstants.Turret.leftAzimuthBigEncoderOffset,
						AdvancedMechanismConstants.Turret.leftAzimuthSmallEncoderOffset,
						AdvancedMechanismConstants.Turret.enc1GearTeethLeft,
						AdvancedMechanismConstants.Turret.enc2GearTeethLeft);

				FlywheelIO flywheelIOLeftTurret = new FlywheelIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftFlywheelID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitFlywheel,
						AdvancedMechanismConstants.Turret.flywheelRatio,
						AdvancedMechanismConstants.Turret.invertFlywheel,
						AdvancedMechanismConstants.Turret.flywheelMaxRPM,
						AdvancedMechanismConstants.Turret.flywheelMOI);
				HoodIO hoodIOLeftTurret = new HoodIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftHoodID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitHood,
						AdvancedMechanismConstants.Turret.minHoodAngle,
						AdvancedMechanismConstants.Turret.maxHoodAngle);

				leftTurret = new Turret(azimuthIOLeftTurret, flywheelIOLeftTurret,
						hoodIOLeftTurret,
						AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter, "LeftTurret");
				kickup = new Kickup(
						new KickupIOSparkBase(AdvancedMechanismConstants.Turret.leftKickupID, Robot.rioCanBus,
								"Kickup", AdvancedMechanismConstants.Turret.currentLimitKickup,
								AdvancedMechanismConstants.Turret.invertKickup, false,
								AdvancedMechanismConstants.Turret.kickerRatio));
				// Right Turret
				AzimuthIO azimuthIORightTurret = new AzimuthIOKrakenFOC(Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightAzimuthID,
						AdvancedMechanismConstants.Turret.rightAzimuthSmallEncoderID,
						AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitAzimuth,
						-AdvancedMechanismConstants.Turret.maxTurretAngle,
						-AdvancedMechanismConstants.Turret.minTurretAngle,
						AdvancedMechanismConstants.Turret.rightAzimuthSmallEncoderOffset,
						AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderOffset,
						AdvancedMechanismConstants.Turret.enc1GearTeethRight,
						AdvancedMechanismConstants.Turret.enc2GearTeethRight);
				FlywheelIO flywheelIORightTurret = new FlywheelIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightFlywheelID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitFlywheel,
						AdvancedMechanismConstants.Turret.flywheelRatio,
						!AdvancedMechanismConstants.Turret.invertFlywheel,
						AdvancedMechanismConstants.Turret.flywheelMaxRPM,
						AdvancedMechanismConstants.Turret.flywheelMOI);
				HoodIO hoodIORightTurret = new HoodIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightHoodID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitHood,
						AdvancedMechanismConstants.Turret.minHoodAngle,
						AdvancedMechanismConstants.Turret.maxHoodAngle);
				rightTurret = new Turret(azimuthIORightTurret, flywheelIORightTurret, hoodIORightTurret,
						AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter, "RightTurret");
				break;
			case SIM:
				GyroSimulation gyroSimulation = null;
				switch (DriveConstants.gyroType) {
					case PIGEON:
						gyroSimulation = GyroSimulation.createPigeon2();
						break;
					case NAVX:
						gyroSimulation = GyroSimulation.createNav2X();
						break;
				}
				switch (DriveConstants.driveType) {
					case SWERVE:
						SwerveModuleSimulation[] moduleSimulations = new SwerveModuleSimulation[4];
						ModuleIO[] moduleIOSims = new ModuleIO[4];
						for (int i = 0; i < 4; i++) {
							switch (DriveConstants.swerveModuleType) {
								case SDSMK4I:
									moduleSimulations[i] = SwerveModuleSimulation
											.getMark4i(DriveConstants.getDriveTrainMotors(1),
													DriveConstants.getDriveTrainMotors(1),
													DriveConstants.gripType.cof, 2)
											.get();
									break;
								case THRIFTYSWERVE:
								case SHIFTING_THIFTYSWERVE:
									moduleSimulations[i] = SwerveModuleSimulation
											.getThriftySwerve(DriveConstants.getDriveTrainMotors(1),
													DriveConstants.getDriveTrainMotors(1),
													DriveConstants.gripType.cof, 2)
											.get();
									break;
								default:
									throw new IllegalArgumentException(
											"Unknown implementation type for module, please check DriveConstants.java!");
							}
							moduleIOSims[i] = new ModuleIOSim(moduleSimulations[i]);
						}

						drivetrainS = new Swerve(new GyroIOSim(gyroSimulation), moduleIOSims[0],
								moduleIOSims[1], moduleIOSims[2], moduleIOSims[3]);
						SwerveDriveSimulation driveSim = new SwerveDriveSimulation(
								DriveConstants.mainRobotProfile.robotMass,
								DriveConstants.kBumperToBumperWidth, DriveConstants.kBumperToBumperLength,
								new SwerveModuleSimulation[] { moduleSimulations[0], moduleSimulations[1],
										moduleSimulations[2], moduleSimulations[3]
								}, DriveConstants.kModuleTranslations, gyroSimulation,
								GeomUtil.apply(startingPose, false), drivetrainS::resetPose);
						fieldSimulation = new Rebuilt2026FieldSimulation(driveSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...
						break;
					case TANK:
						final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(
								DriveConstants.kChassisWidth);
						final GyroIOSim tankGyroIOSim = new GyroIOSim(gyroSimulation);
						TankIOSim tankIOSim = new TankIOSim(tankGyroIOSim);
						drivetrainS = new Tank(tankIOSim);
						TankDriveSimulation tankSim = new TankDriveSimulation(DriveConstants.mainRobotProfile,
								gyroSimulation,
								diffKinematics,
								GeomUtil.apply(startingPose, false),
								(Tank) drivetrainS,
								tankIOSim,
								drivetrainS::resetPose);
						fieldSimulation = new Rebuilt2026FieldSimulation(tankSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...

						break;
					default:
						final MecanumDriveKinematics mechKinematics = new MecanumDriveKinematics(
								DriveConstants.kModuleTranslations[0],
								DriveConstants.kModuleTranslations[1],
								DriveConstants.kModuleTranslations[2],
								DriveConstants.kModuleTranslations[3]);
						final GyroIOSim mecanumGyroIOSim = new GyroIOSim(gyroSimulation);
						MecanumIOSim mecanumIOSim = new MecanumIOSim(mecanumGyroIOSim);
						drivetrainS = new Mecanum(mecanumIOSim);
						MecanumDriveSimulation mecanumSim = new MecanumDriveSimulation(DriveConstants.mainRobotProfile,
								gyroSimulation,
								mechKinematics,
								GeomUtil.apply(startingPose, false),
								(Mecanum) drivetrainS,
								mecanumIOSim,
								drivetrainS::resetPose);
						fieldSimulation = new Rebuilt2026FieldSimulation(mecanumSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...
						break;
				}

				/*
				 * visionS = new Vision(() -> getSelectedAprilTagLayout(),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "IntakeCam",0,
				 * VisionConstants.cameras[0]),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackRightCam",1,
				 * VisionConstants.cameras[1]),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackLeftCam",2,
				 * VisionConstants.cameras[2]));
				 */

				visionS = new Vision(() -> getSelectedAprilTagLayout(),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "IntakeCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[0].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "BackRightCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[1].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "BackLeftCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[2].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()));
				toggles = new Toggles(new TogglesIONetworkTables());
				/*
				 * hang = new Hang(new Climber(new ClimberIOSim(DCMotor.getKrakenX44Foc(1),
				 * "Climber",
				 * SimpleMechanismConstants.Climber.climbReductionToClimbRollers,
				 * SimpleMechanismConstants.Climber.climbMOI)),
				 * new WedgeArmIOSim());
				 */
				intake = new Intake(new ArmIOSim(), new Indexer(new IndexerIOSim(DCMotor.getKrakenX44Foc(1), "Indexer",
						IntakeConstants.intakeReductionToIndexerRollers, IntakeConstants.intakeMOI)),
						new FrontRollers(
								new FrontRollersIOSim(DCMotor.getKrakenX44Foc(1), IntakeConstants.frontRollersName,
										IntakeConstants.frontRollersReduction, IntakeConstants.frontRollersMOI)));
				kickup = new Kickup(new KickupIOSim(AdvancedMechanismConstants.Turret.kickupMotor, "Kickup",
						AdvancedMechanismConstants.Turret.kickerRatio,

						AdvancedMechanismConstants.Turret.kickupMOI));
				leftTurret = new Turret(
						new AzimuthIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.leftAzimuthID,
								AdvancedMechanismConstants.Turret.leftAzimuthBigEncoderID,
								AdvancedMechanismConstants.Turret.leftAzimuthSmallEncoderID,
								AdvancedMechanismConstants.Turret.leftName + "Azimuth",
								AdvancedMechanismConstants.Turret.currentLimitAzimuth,
								AdvancedMechanismConstants.Turret.minTurretAngle,
								AdvancedMechanismConstants.Turret.maxTurretAngle,
								AdvancedMechanismConstants.Turret.enc1GearTeethLeft,
								AdvancedMechanismConstants.Turret.enc2GearTeethLeft),
						new FlywheelIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.leftFlywheelID,
								AdvancedMechanismConstants.Turret.leftName,
								AdvancedMechanismConstants.Turret.currentLimitFlywheel,
								AdvancedMechanismConstants.Turret.flywheelRatio,
								AdvancedMechanismConstants.Turret.invertFlywheel,
								AdvancedMechanismConstants.Turret.flywheelMaxRPM,
								AdvancedMechanismConstants.Turret.flywheelMOI),
						new HoodIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.leftHoodID,
								AdvancedMechanismConstants.Turret.leftName + "Hood"),

						AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter, "LeftTurret");
				rightTurret = new Turret(
						new AzimuthIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.rightAzimuthID,
								AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID,
								AdvancedMechanismConstants.Turret.rightAzimuthSmallEncoderID,
								AdvancedMechanismConstants.Turret.rightName + "Azimuth",
								AdvancedMechanismConstants.Turret.currentLimitAzimuth,
								-AdvancedMechanismConstants.Turret.maxTurretAngle,
								-AdvancedMechanismConstants.Turret.minTurretAngle,
								AdvancedMechanismConstants.Turret.enc1GearTeethRight,
								AdvancedMechanismConstants.Turret.enc2GearTeethRight),
						new FlywheelIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.rightFlywheelID,
								AdvancedMechanismConstants.Turret.rightName,
								AdvancedMechanismConstants.Turret.currentLimitFlywheel,
								AdvancedMechanismConstants.Turret.flywheelRatio,
								AdvancedMechanismConstants.Turret.invertFlywheel,
								AdvancedMechanismConstants.Turret.flywheelMaxRPM,
								AdvancedMechanismConstants.Turret.flywheelMOI),
						new HoodIOSim(
								Robot.rioCanBus,
								AdvancedMechanismConstants.Turret.rightHoodID,
								AdvancedMechanismConstants.Turret.rightName + "Hood"),

						AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter, "RightTurret");
				/*
				 * autoCommands.addAll(Arrays.asList(
				 * new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
				 * Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
				 * new Pair<String, Command>("SmartShoot", Commands.none()),
				 * new Pair<String, Command>("SmartIntake", Commands.none())
				 * new Pair<String, Command>("BranchGrabbingGamePiece",
				 * new BranchAuto("Shoot",
				 * new Pose2d(7.4, 5.8, new Rotation2d()), 4))
				 * // new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)),
				 * //NEEDS
				 * // A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				 * // new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS,
				 * false,new
				 * // Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				 * // new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				 * ));
				 */
				System.out.println("SIM SETUP DONE!");
				break;
			default:
				switch (DriveConstants.driveType) {
					case SWERVE:
						drivetrainS = new Swerve(new GyroIO() {
						}, new ModuleIO() {
						},
								new ModuleIO() {
								}, new ModuleIO() {
								}, new ModuleIO() {
								});
						break;
					case TANK:
						drivetrainS = new Tank(new TankIO() {
						});
						break;
					case MECANUM:
						drivetrainS = new Mecanum(new MecanumIO() {
						});
				}

				visionS = new Vision(() -> getSelectedAprilTagLayout(), new VisionIO() {
				}, new VisionIO() {
				},
						new VisionIO() {
						}); // MUST be same number of cameras as in real robot
				toggles = new Toggles(new TogglesIO() {
				});
				/*
				 * hang = new Hang(new Climber(new ClimberIO() {
				 * }), new WedgeArmIO() {
				 * });
				 */
				intake = new Intake(new ArmIO() {
				}, new Indexer(new IndexerIO() {
				}), new FrontRollers(new FrontRollersIO() {
				}));
				kickup = new Kickup(new KickupIO() {
				});

				leftTurret = new Turret(new AzimuthIO() {
				}, new FlywheelIO() {
				}, new HoodIO() {
				}, AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter,
						"LeftTurret");
				rightTurret = new Turret(new AzimuthIO() {
				}, new FlywheelIO() {
				}, new HoodIO() {
				}, AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter, "RightTurret");

		}
		NamedCommands.registerCommand(
				"Intake", Commands.run(() -> intake.setGoal(Goal.INTAKE_GROUND), intake)
						.finallyDo(() -> intake.setGoal(Goal.INTAKE_OUTER_IDLE)));
		NamedCommands.registerCommand(
				"ShootWithIntakeOut", buildShootTurretsHubIntakeOutCommand());
		NamedCommands.registerCommand(
				"Shoot", buildShootTurretsHubIntakeOutCommand());
		// NamedCommands.registerCommand("Hang", buildHangCommand());
		NamedCommands.registerCommand("UseDrive", Commands.run(() -> {
			drivetrainS.stopModules();
		}, drivetrainS).withName("UseDrive").ignoringDisable(true));
		NamedCommands.registerCommand("WarmShooter", buildTargetHubBothCommand());
		drivetrainS.resetPose(startingPose);
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		Pathfinding.setPathfinder(pathFinder);
		// algaeScorer.setDefaultCommand(new AlgaeScorerC(algaeScorer));
		// superStructureNotifier = new Notifier(superStructure::periodic);
		// superStructureNotifier.startPeriodic(.01);
		// Add all the auto commands to the auto builder

		// Make sure to watch your flipped poses. Our custom DriveToPose and all of
		// those do NOT auto flip for red
		precalculateAllStartAndEndChoreos();

		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}

		CommandScheduler.getInstance()
				.schedule(new PathfindingCommand(new Pose2d(15.0, 4.0, Rotation2d.k180deg),
						new PathConstraints(8, 11, 4, 4), () -> new Pose2d(1.5, 4, Rotation2d.kZero),
						ChassisSpeeds::new, (speeds, feedforwards) -> {
						},
						new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
								new PIDConstants(5.0, 0.0, 0.0)),
						DriveConstants.mainConfig)
						.andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
						.ignoringDisable(true).finallyDo(() -> RobotContainer.field.getObject("target pose")
								.setPose(new Pose2d(-50, -50, new Rotation2d()))));
		/*
		 * if (!leds.gifFound(ImageStates.debug)) {
		 * Logger.recordOutput("LEDS/Main",
		 * "No images found for " + ImageStates.debug.name());
		 * }
		 */

		if (!AutoBuilder.isConfigured()) {
			throw new RuntimeException("AutoBuilder was not configured before attempting to build an auto chooser");
		}

		JukeboxUtil jukebox = new JukeboxUtil();
		for (ParentDevice device :

		getOrchestraDevices()) {
			jukebox.addTalon(device);
		}
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		autoChooser.addDefaultOption("DynamicPathing",
				Commands.defer(() -> PosePlotterUtil.getAuto(), Set.of()));
		if (drivetrainS instanceof Swerve) {
			Command orientBeforeData = ((Swerve) drivetrainS).orientModules(Swerve.getCircleOrientations());
			autoChooser.addOption("Wheel Radius Characterization",
					orientBeforeData
							.andThen(new WheelRadiusCharacterization(drivetrainS,
									WheelRadiusCharacterization.Direction.CLOCKWISE))
							.withName("DRIVE wheel radius characterization"));
		} else {
			autoChooser.addOption("Wheel Radius Characterization",
					new WheelRadiusCharacterization(drivetrainS,
							WheelRadiusCharacterization.Direction.CLOCKWISE)
							.withName("DRIVE wheel radius characterization"));
		}
		autoChooser.addOption("Drive Static Characterization",
				new StaticCharacterization(drivetrainS, drivetrainS::runCharacterization,
						drivetrainS::getCharacterizationVelocity)
						.finallyDo(drivetrainS::endCharacterization)
						.withName("Drive Static Characterization"));
		autoChooser.addOption("Drive FeedForward Characterization",
				new FeedForwardCharacterization(drivetrainS, drivetrainS::runCharacterization,
						drivetrainS::getCharacterizationVelocity, () -> false) // NEVER automatically end. MUST disable
																				// to end.
						.finallyDo(drivetrainS::endCharacterization)
						.withName("Drive FeedForward Characterization"));

		// autos for tuning
		autoChooser.addOption("Intake PID Char",
				new RoughPIDCharacterization(intake, (volts) -> intake.runCharacterization(volts),
						intake::getCharacterizationMeasurement, intake::getCharVelocity,
						IntakeConstants.armMinAngleRads, IntakeConstants.armMaxAngleRads, Units.degreesToRadians(10),
						Units.degreesToRadians(120), 3, 20).beforeStarting(Commands.waitSeconds(3))
						.withName("Intake PID Characterization"));
		autoChooser.addOption("Intake FeedForward Characterization",
				new FeedForwardCharacterization(intake, (volts) -> intake.runCharacterization(volts),
						intake::getCharacterizationMeasurement, () -> false).beforeStarting(Commands.waitSeconds(3))
						.withName("Intake FeedForward Characterization"));
		SmartDashboard.putData(field);

		// Configure the trigger bindings
		configureBindings();
		addNTCommands();
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		// Some condition that should decide if we want to override rotation
		return angleOverrider;
	}

	private void configureBindings() {
		Trigger povUp = driveController.pov(0);
		Trigger povRight = driveController.pov(90);
		Trigger povDown = driveController.pov(180);
		Trigger povLeft = driveController.pov(270);

		Trigger manualTurretControl = povUp.or(povRight).or(povDown).or(povLeft);
		Trigger inTeleOp = new Trigger(() -> DriverStation.isTeleop() && DriverStation.isEnabled());
		Trigger inScoreArea = new Trigger(() -> GeomUtil.applyX(drivetrainS.getPose().getX()) < 4.4); // when red, we
																										// are at 12,
																										// flipping
																										// across gets
																										// to below 4.4
		Trigger inOpponentArea = new Trigger(() -> GeomUtil.applyX(drivetrainS.getPose().getX()) > 12.0);
		Trigger beyondLeftTrench = new Trigger(() -> GeomUtil.applyY(drivetrainS.getPose().getY()) > 5.5);
		Trigger beyondCenter = new Trigger(() -> GeomUtil.applyY(drivetrainS.getPose().getY()) > 4);
		Trigger beforeRightTrench = new Trigger(() -> GeomUtil.applyY(drivetrainS.getPose().getY()) < 2.3);
		final double trenchHardLockMeters = 0.6;
		final double hoodSafeDownDeg = 13.0;
		final double hoodDownRateDegPerSec = 38.0;
		final double trenchUnlockDebounceSec = 0.4;
		BooleanSupplier nearAnyTrenchRaw = () -> {
			Translation2d robotPos = drivetrainS.getPose().getTranslation();
			ChassisSpeeds fieldSpeeds = drivetrainS.getFieldChassisSpeeds();
			Translation2d fieldVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond,
					fieldSpeeds.vyMetersPerSecond);
			double maxHoodDeg = Math.max(Math.toDegrees(leftTurret.hoodAngle()),
					Math.toDegrees(rightTurret.hoodAngle()));
			double hoodSecondsToDown = Math.max(0.0, (maxHoodDeg - hoodSafeDownDeg) / hoodDownRateDegPerSec);
			Translation2d[] trenchCenters = new Translation2d[] {
					FieldConstants.LeftTrench.openingCenter,
					FieldConstants.RightTrench.openingCenter,
					GeomUtil.apply(FieldConstants.LeftTrench.openingCenter, true),
					GeomUtil.apply(FieldConstants.RightTrench.openingCenter, true)
			};
			for (Translation2d center : trenchCenters) {
				Translation2d robotToTrench = center.minus(robotPos);
				double distanceToTrench = robotToTrench.getNorm();
				if (distanceToTrench < 1e-6) {
					return true;
				}
				Translation2d towardUnit = robotToTrench.div(distanceToTrench);
				double towardSpeedMps = Math.max(0.0,
						fieldVelocity.getX() * towardUnit.getX() + fieldVelocity.getY() * towardUnit.getY());
				double dynamicTrenchDistanceM = trenchHardLockMeters + towardSpeedMps * hoodSecondsToDown;
				if (distanceToTrench < dynamicTrenchDistanceM) {
					return true;
				}
			}
			return false;
		};
		Debouncer trenchUnlockDebouncer = new Debouncer(trenchUnlockDebounceSec, Debouncer.DebounceType.kFalling);
		Trigger nearAnyTrench = new Trigger(() -> trenchUnlockDebouncer.calculate(nearAnyTrenchRaw.getAsBoolean()));
		Trigger hoodAboveSafeAngle = new Trigger(
				() -> leftTurret.isHoodAboveDegrees(14.0) || rightTurret.isHoodAboveDegrees(14.0));
		Command targetHubBoth = buildTargetHubBothCommand();
		Command shootTurrets = buildShootTurretsCommand();

		var targetSplitTrenches = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.OVER_NEUTRAL_ZONE);
			rightTurret.setPresetTarget(Turret.PresetTarget.OVER_NEUTRAL_ZONE);
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
		}, leftTurret, rightTurret);

		var targetBothLeftTrench = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
		}, leftTurret, rightTurret);

		var targetBothRightTrench = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
		}, leftTurret, rightTurret);
		var targetBothOverNeutral = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.OVER_NEUTRAL_ZONE);
			rightTurret.setPresetTarget(Turret.PresetTarget.OVER_NEUTRAL_ZONE);
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
		}, leftTurret, rightTurret);
		var shootTurretsWhileIntaking = Commands.runOnce(() -> {
			shootTimer.restart();
		}).andThen(Commands.run(() -> {
			leftTurret.setGoal(Turret.Goal.SHOOTING);
			rightTurret.setGoal(Turret.Goal.SHOOTING);
			applyShootCycleGoals(Goal.INTAKE_GROUND_SHOOT, shootTimer.hasElapsed(0.5));
		}, leftTurret, rightTurret, intake, kickup).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
			intake.setGoal(Goal.INTAKE_OUTER_IDLE);
			kickup.setGoal(Kickup.Goal.IDLING);
			shootTimer.stop();
		}));
		// Auto factory setup
		touchboardAutoFactory = new TouchboardAutoFactory(pathFinder, drivetrainS,
				() -> new AutoIntake(drivetrainS, intake, CameraID.INTAKE_CAM), Set.of(intake, drivetrainS),
				() -> buildTargetHubBothCommand().andThen(buildShootTurretsCommand()).withName("Shoot Turrets Auto"),
				Set.of(leftTurret, rightTurret, kickup, intake));

		// Start of actual DRIVER bindings
		// Start = zero chassis
		// Select = zero robot
		// Left Stick Button = orient modules to circle for pushing
		// Right Stick Button = play megolovania because why not
		// POV-Up / BR paddle = vomit
		// POV-Right / TR paddle = manual turret control for aiming at right trench
		// POV-Down / BL paddle = vomit
		// POV-Left / TL paddle = manual turret control for aiming at left trench
		// Left Trigger = auto align THRU the trench with velocity
		// Right Trigger = fire while aiming at target
		// Left Bumper = hold to intake from ground
		// Right Bumper = hold to agitate intake (oscillate arm 0-25deg)
		// A button = vomit
		// B button = clear logged shots
		// Y button = (reserved)
		// X button = jackhammer intake/indexer/Vindexer
		// NOTE: Intake defaults DOWN (out). Manip X holds it UP (stow).

		startButtonDrive
				.onTrue(new InstantCommand(() -> {
					System.out.println("Zeroing Bot");
					drivetrainS.zeroHeading();
					// drivetrainS.resetPose(GeomUtil.apply(startingPose.get(), false));
				}));
		selectButtonDrive
				.onTrue(new InstantCommand(() -> {
					System.out.println("Zeroing Intake/Stopping Turrets/Hoods");
					intake.zero();
					leftTurret.zeroHood();
					rightTurret.zeroHood();
					leftTurret.setGoal(Turret.Goal.IDLE);
					rightTurret.setGoal(Turret.Goal.IDLE);

				}));
		leftStickButtonDrive.onTrue(drivetrainS.orientModules(Swerve.getXOrientations()));
		leftStickButtonDrive.onFalse(Commands.runOnce(() -> drivetrainS.stopModules(), drivetrainS));
		rightStickButtonDrive.onTrue(new OrchestraC("speed").withName("Play Megolovania"));
		// Test Commands
		aButtonDrive.whileTrue(Commands.run(() -> {
			intake.setGoal(Goal.VOMITING);
			kickup.setGoal(Kickup.Goal.VOMITING);
			// flywheel go to 5000 rpm
			// leftTurret.setCharHoodPos(0);(4.1);
			// leftTurret.setCharHoodPos(Units.degreesToRadians(12));
			// rightTurret.setCharHoodPos(Units.degreesToRadians(12));
			// intake.setGoal(Goal.STOW);
			// leftTurret.setCharTurretPos(-1.49);
			// hang.setGoal(HangState.STOWED);
		}));
		bButtonDrive.onTrue(Commands.runOnce(() -> {
			leftTurret.clearLoggedShots();
			rightTurret.clearLoggedShots();
		}));
		yButtonDrive.onTrue(Commands.runOnce(() -> {
			leftTurret.enterShotTuning();
			rightTurret.enterShotTuning();
		}));
		// Climber controls
		// aButtonDrive.onTrue(Commands.either(Commands.runOnce(() ->
		// hang.setGoal(HangState.EXTENDED)), Commands.runOnce(() ->
		// hang.setGoal(HangState.STOWED)), () -> hang.getHangState() ==
		// HangState.STOWED));
		// yButtonDrive.whileTrue(Commands.run(() ->
		// hang.setGoal(HangState.MOVING_UP)));
		// bButtonDrive.whileTrue(Commands.run(() ->
		// hang.setGoal(HangState.MOVING_DOWN)));
		// Intake controls (gated: disabled when manip left trigger is held for manual
		// voltage override)
		leftBumperDrive.and(rightTriggerDriveFull.negate()).and(manipLeftTrigger.negate()).whileTrue(
				Commands.run(() -> intake.setGoal(Goal.INTAKE_GROUND))
						.finallyDo(() -> intake.setGoal(Goal.INTAKE_OUTER_IDLE)));
		rightBumperDrive.and(rightTriggerDriveFull.negate()).and(manipLeftTrigger.negate()).whileTrue(
				(Commands.runOnce(() -> intake.setGoal(Goal.AGITATING), intake).andThen(Commands.waitSeconds(999)))
						.finallyDo(() -> intake.setGoal(Goal.INTAKE_OUTER_IDLE)));
		// leftBumperDrive.onTrue(Commands.runOnce(()));
		xButtonDrive.and(manipLeftTrigger.negate()).whileTrue(Commands.either(
				Commands.run(() -> {
					intake.setGoal(Goal.JACKHAMMERING_OUT);
					kickup.setGoal(frc.robot.subsystems.Turret.kickup.Kickup.Goal.JACKHAMMER);
				}, intake, kickup)
						.finallyDo(() -> {
							intake.setGoal(Goal.INTAKE_OUTER_IDLE);
							kickup.setGoal(frc.robot.subsystems.Turret.kickup.Kickup.Goal.IDLING);
						}),
				Commands.run(() -> {
					intake.setGoal(Goal.JACKHAMMERING_IN);
					kickup.setGoal(frc.robot.subsystems.Turret.kickup.Kickup.Goal.JACKHAMMER);
				}, intake, kickup)
						.finallyDo(() -> {
							intake.setGoal(Goal.INTAKE_OUTER_IDLE);
							kickup.setGoal(frc.robot.subsystems.Turret.kickup.Kickup.Goal.IDLING);
						}),
				() -> intake.isIntakeDeployed())); // jackhammer

		// Auto Driving To Alliance
		// If we're before the right, go to the right trench with 3 m/s going DOWN (up
		// if on red).
		// If we're after the left, go to the left trench with 3 m/s going "DOWN" (up if
		// on red).
		// If we're between, just MadMax it over the bumps, using normal pathfinding.
		AtomicBoolean committedToMadMax = new AtomicBoolean(false);
		Trigger trenchesAllowed = new Trigger(() -> !committedToMadMax.get());

		leftTriggerDriveFull
				.and((beforeRightTrench.and(trenchesAllowed))
						.or(beyondCenter.negate().and(inScoreArea).and(trenchesAllowed)))
				.whileTrue(Commands.defer(() -> PathFinder.goToAutoPilotPoseNoHardLineup(pathFinder,
						new APTarget(GeomUtil.apply(new Pose2d(4.609, .639, new Rotation2d()), false)).withVelocity(4)
								.withEntryAngle(new Rotation2d(Math.PI / 2)),
						drivetrainS, () -> DriveConstants.pathConstraints, Units.inchesToMeters(6)),
						Set.of(drivetrainS)));
		leftTriggerDriveFull
				.and(((beyondLeftTrench).and(trenchesAllowed)).or(beyondCenter.and(inScoreArea).and(trenchesAllowed)))
				.whileTrue(Commands.defer(() -> PathFinder.goToAutoPilotPoseNoHardLineup(pathFinder,
						new APTarget(GeomUtil.apply(new Pose2d(4.605, 7.404, new Rotation2d()), false)).withVelocity(4)
								.withEntryAngle(new Rotation2d(Math.PI / 2)),
						drivetrainS, () -> DriveConstants.pathConstraints, Units.inchesToMeters(6)),
						Set.of(drivetrainS)));
		leftTriggerDriveFull
				.and((beforeRightTrench.negate().and(beyondLeftTrench.negate())).or(trenchesAllowed.negate()))
				.and(inScoreArea.negate())
				.whileTrue(
						Commands.runOnce(() -> committedToMadMax.set(true)).andThen(Commands.defer(
								() -> PathFinder.goToAutoPilotPose(pathFinder,
										new APTarget(GeomUtil.apply(new Pose2d(3.6, drivetrainS.getPose().getY(),
												new Rotation2d()), false)).withVelocity(4)
												.withEntryAngle(new Rotation2d(Math.PI / 2)),
										drivetrainS,
										() -> DriveConstants.pathConstraints, 1, Units.inchesToMeters(16)),
								Set.of(drivetrainS))));
		leftTriggerDriveFull.onFalse(Commands.runOnce(() -> committedToMadMax.set(false)));
		nearAnyTrench.onTrue(Commands.runOnce(() -> {
			leftTurret.setTrenchHoodLock(true);
			rightTurret.setTrenchHoodLock(true);
		}));
		nearAnyTrench.onFalse(Commands.runOnce(() -> {
			leftTurret.setTrenchHoodLock(false);
			rightTurret.setTrenchHoodLock(false);
		}));
		hoodAboveSafeAngle.whileTrue(Commands.startEnd(
				() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
				() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
		// Turret Controls
		rightTriggerDriveFull.and(leftBumperDrive.negate()).and(rightBumperDrive.negate()).and(xButtonDrive.negate())
				.whileTrue(shootTurrets);
		rightTriggerDriveFull.and(leftBumperDrive).and(xButtonDrive.negate()).whileTrue(shootTurretsWhileIntaking);
		// Right trigger + right bumper = shoot while agitating intake
		rightTriggerDriveFull.and(rightBumperDrive).and(leftBumperDrive.negate()).and(xButtonDrive.negate()).whileTrue(
				Commands.runOnce(() -> {
					shootTimer.restart();
				}).andThen(Commands.run(() -> {
					leftTurret.setGoal(Turret.Goal.SHOOTING);
					rightTurret.setGoal(Turret.Goal.SHOOTING);
					// Agitate arm while cycling jackhammer on rollers/kickup
					intake.setGoal(Goal.AGITATING);
					if (isInJackhammerPhase()) {
						kickup.setGoal(Kickup.Goal.JACKHAMMER);
					} else if (shootTimer.hasElapsed(0.5)) {
						kickup.setGoal(Kickup.Goal.SHOOTING);
					} else {
						kickup.setGoal(Kickup.Goal.IDLING);
					}
				}, leftTurret, rightTurret, kickup, intake).finallyDo(() -> {
					leftTurret.setGoal(Turret.Goal.AIMING);
					rightTurret.setGoal(Turret.Goal.AIMING);
					intake.setGoal(Goal.INTAKE_OUTER_IDLE);
					kickup.setGoal(Kickup.Goal.IDLING);
					shootTimer.stop();
				})));
		povUp.whileTrue(Commands.run(() -> {
			intake.setGoal(Goal.VOMITING);
			kickup.setGoal(Kickup.Goal.VOMITING);
		}, intake, kickup).finallyDo(() -> {
			intake.setGoal(Goal.INTAKE_OUTER_IDLE);
			kickup.setGoal(Kickup.Goal.IDLING);
		}));
		/*povDown.whileTrue(Commands.run(() -> {
			intake.setGoal(Goal.VOMITING);
			kickup.setGoal(Kickup.Goal.VOMITING);
		}, intake, kickup).finallyDo(() -> {
			intake.setGoal(Goal.INTAKE_OUTER_IDLE);
			kickup.setGoal(Kickup.Goal.IDLING);
		}));*/
		povDown.onTrue(targetBothOverNeutral);
		povLeft.onTrue(targetHubBoth);
		povRight.onTrue(targetSplitTrenches);
		// Manip Controls
		// Manip X = hold intake up (stow), release to go back down
		manipXButton.whileTrue(
				Commands.run(() -> intake.setGoal(Goal.STOW), intake)
						.finallyDo(() -> intake.setGoal(Goal.INTAKE_OUTER_IDLE)));
		Trigger manipManualTurret = new Trigger(
				() -> Math.abs(manipController.getLeftX()) > .1 || Math.abs(manipController.getLeftY()) > .1);
		// manipManualTurret.whileTrue(());
		manipManualTurret.whileTrue(Commands.run(() -> {
			// Read stick
			double x = -manipController.getLeftX();
			double y = manipController.getLeftY();
			double mag = Math.hypot(x, y);
			if (mag <= 0.07) {
				return; // deadband
			}

			double stickAngleField = Math.atan2(y, x) - Math.PI / 2.0;
			if (stickAngleField > Math.PI)
				stickAngleField -= 2.0 * Math.PI;
			else if (stickAngleField < -Math.PI)
				stickAngleField += 2.0 * Math.PI;

			// Map magnitude [deadband..1] to distance [1m .. 10m]
			double magNorm = Math.max(0.0, Math.min(1.0, (mag - 0.07) / (1.0 - 0.07)));
			double rangeMeters = 1.0 + magNorm * (4.0 - 1.0);

			Pose2d robotPose = drivetrainS.getPose();
			Translation2d targetField = new Translation2d(
					robotPose.getX() + rangeMeters * Math.cos(stickAngleField),
					robotPose.getY() + rangeMeters * Math.sin(stickAngleField));

			ShotCalculator calc = ShotCalculator.getInstance();

			// Left turret
			ShootingParameters leftParams = calc.getParameters(
					targetField, leftTurret.getRobotToTurret(), ShotCalculator.HUB_PROFILE, 0);
			leftTurret.setCharTurretPos(leftParams.turretAngle().getRadians());
			leftTurret.setCharHoodPos(leftParams.hoodAngle());
			leftTurret.setCharRPM(leftParams.flywheelSpeed() * 60.0 / (2.0 * Math.PI)); // rad/s -> RPM

			// Right turret
			ShootingParameters rightParams = calc.getParameters(
					targetField, rightTurret.getRobotToTurret(), ShotCalculator.HUB_PROFILE, 0);
			rightTurret.setCharTurretPos(rightParams.turretAngle().getRadians());
			rightTurret.setCharHoodPos(rightParams.hoodAngle());
			rightTurret.setCharRPM(rightParams.flywheelSpeed() * 60.0 / (2.0 * Math.PI)); // rad/s -> RPM
		}));
		manipLeftTrigger.whileTrue(Commands.run(() -> {
			double stickY = manipController.getLeftY();
			double volts = -stickY * 3.0;
			intake.runCharacterization(volts);
		}, intake).finallyDo(() -> {
			intake.holdAtCurrentPosition();
		}));
		manipRightTrigger.whileTrue(Commands.run(() -> {
			double stickY = manipController.getLeftY();
			double volts = stickY * 3.0;
			intake.runCharacterization(volts);
		}, intake).finallyDo(() -> {
			intake.holdAtCurrentPosition();
		}));
		manipAButton.whileTrue(Commands.startEnd(
				() -> {
					rightTurret.setManualHoodRezeroHeld(true);
					leftTurret.setManualHoodRezeroHeld(true);
				},
				() -> {
					rightTurret.setManualHoodRezeroHeld(false);
					leftTurret.setManualHoodRezeroHeld(false);
				}));
		manipUpPov.onTrue(Commands.runOnce(() -> {
			rightTurret.offsetDistance(.125);
			leftTurret.offsetDistance(.125);
		}));
		manipDownPov.onTrue(Commands.runOnce(() -> {
			rightTurret.offsetDistance(-.125);
			leftTurret.offsetDistance(-.125);
		}));
		manipYButton.whileTrue(Commands.runOnce(() -> {
			shootTimer.restart();
			kickup.setGoal(Kickup.Goal.IDLING);
		}).andThen(Commands.run(() -> {
			rightTurret.setGoal(Turret.Goal.SHOOTING_FROM_HUB);
			leftTurret.setGoal(Turret.Goal.SHOOTING_FROM_HUB);
			applyShootCycleGoals(Goal.INTAKE_GROUND_SHOOT, shootTimer.hasElapsed(0.5));
		}, leftTurret, rightTurret, kickup, intake).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
			kickup.setGoal(Kickup.Goal.IDLING);
			intake.setGoal(Intake.Goal.INTAKE_OUTER_IDLE);
			shootTimer.stop();
		})));
		// Automatic Turret Controls -- DISABLED UNTIL TUNING COMPLETE!

		// manualTurretControl.negate().and(inScoreArea).and(inTeleOp).whileTrue(targetHubBoth);
		/*
		 * manualTurretControl.negate().and(inScoreArea.negate()).and(inOpponentArea.
		 * negate()).and(beforeRightTrench).and(inTeleOp)
		 * .whileTrue(targetBothRightTrench);
		 * manualTurretControl.negate().and(inScoreArea.negate()).and(inOpponentArea.
		 * negate()).and(beyondLeftTrench).and(inTeleOp)
		 * .whileTrue(targetBothLeftTrench);
		 */
		/*
		 * manualTurretControl.negate().and(inScoreArea.negate()).and(inOpponentArea.
		 * negate()).and(inTeleOp)
		 * .and(beyondLeftTrench.negate()).and(beforeRightTrench.negate()).whileTrue(
		 * targetSplitTrenches);
		 */
		/*
		 * manualTurretControl.negate().and(inOpponentArea).and(inTeleOp).whileTrue(
		 * targetBothOverNeutral);
		 */

		// - If in score area AND not manually holding POV: aim both turrets at hub
		/*
		 * inScoreArea.and(manualTurretControl.negate())
		 * .whileTrue(aimHubBoth);
		 */

		// - If NOT in score area AND not holding POV: idle both turrets
		/*
		 * inScoreArea.negate().and(manualTurretControl.negate())
		 * .onTrue(idleBoth);
		 */
		if (Constants.currentMode == Mode.SIM) {
			/*
			 * testDPadUp.onTrue(new InstantCommand(() -> {
			 * try {
			 * System.out.println("Creating Algae");
			 * fieldSimulation.addGamePiece(new
			 * Reefscape2025FieldObjects.AlgaeBallOnManipulator(
			 * Logger.getTimestamp(), 999,
			 * fieldSimulation.getMainDriveSimulation().getPose3d()));
			 * } catch (Exception e) {
			 * System.out.println("Failed to Create Algae");
			 * }
			 * }));
			 * testDPadDown.onTrue(new InstantCommand(() -> {
			 * try {
			 * System.out.println("Creating Coral");
			 * fieldSimulation.addGamePiece(new
			 * Reefscape2025FieldObjects.ReefscapeCoralOnManipulator());
			 * // superStructure.setCoralGamepieceState(CoralGamepieceState.HOPPER_STAGED);
			 * } catch (Exception e) {
			 * System.out.println("Failed to Create Algae");
			 * }
			 * }));
			 */
		}
	}

	// Interface for command factories
	public interface CommandFactory {
		Command generate();
	}

	public static AprilTagLayoutType getSelectedAprilTagLayout() {
		return AprilTagLayoutType.OFFICIAL;
	}

	/**
	 * Returns true when the shoot cycle is in the jackhammer phase.
	 * Call every loop during any shooting command to auto-jackhammer.
	 * Uses FPGA timestamp so all commands share the same global cycle.
	 */
	private boolean isInJackhammerPhase() {
		double cyclePeriod = shootCycleShootSec.get() + shootCycleJackhammerSec.get();
		double tInCycle = Timer.getFPGATimestamp() % cyclePeriod;
		return tInCycle >= shootCycleShootSec.get();
	}

	/**
	 * Sets intake and kickup goals for the current shoot cycle phase.
	 * During the shoot phase: intake gets shootGoal, kickup gets SHOOTING (if allowed).
	 * During the jackhammer phase: intake jackhammers, kickup jackhammers.
	 *
	 * @param shootGoal     the intake goal to use during normal shooting (e.g. SHOOTING or INTAKE_GROUND_SHOOT)
	 * @param allowKickup   whether the kickup is allowed to shoot (e.g. after shootTimer delay)
	 */
	private void applyShootCycleGoals(Goal shootGoal, boolean allowKickup) {
		if (isInJackhammerPhase()) {
			// Jackhammer phase — cycle intake and kickup
			if (intake.isIntakeDeployed()) {
				intake.setGoal(Goal.JACKHAMMERING_OUT);
			} else {
				intake.setGoal(Goal.JACKHAMMERING_IN);
			}
			kickup.setGoal(Kickup.Goal.JACKHAMMER);
		} else {
			// Normal shooting phase
			intake.setGoal(shootGoal);
			if (allowKickup) {
				kickup.setGoal(Kickup.Goal.SHOOTING);
			} else {
				kickup.setGoal(Kickup.Goal.IDLING);
			}
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	private Command buildTargetHubBothCommand() {
		return Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
		}, leftTurret, rightTurret);
	}

	private Command buildShootTurretsHubIntakeOutCommand() {
		return (buildTargetHubBothCommand().andThen(Commands.runOnce(() -> {
			shootTimer.restart();
			kickup.setGoal(Kickup.Goal.IDLING);
		}).andThen(Commands.run(() -> {
			rightTurret.setGoal(Turret.Goal.SHOOTING);
			leftTurret.setGoal(Turret.Goal.SHOOTING);
			applyShootCycleGoals(Goal.INTAKE_GROUND_SHOOT, shootTimer.hasElapsed(0.5));
		}, leftTurret, rightTurret, kickup, intake).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
			kickup.setGoal(Kickup.Goal.IDLING);
			intake.setGoal(Intake.Goal.INTAKE_OUTER_IDLE);
			shootTimer.stop();
		})))).withName("Shoot Turrets with Intake Out");
	}

	/*
	 * private Command buildHangCommand() {
	 * return Commands.defer(() -> Commands.sequence(Commands.runOnce(() -> {
	 * hang.setGoal(HangState.EXTENDED);
	 * }).andThen(
	 * //go into wall, then climb up, then we ball TODO.
	 * )), Set.of(hang, drivetrainS)).withName("Auto Hang");
	 * }
	 */

	private Command buildShootTurretsHubIntakeInCommand() {
		return (buildTargetHubBothCommand().andThen(Commands.runOnce(() -> {
			shootTimer.restart();
			kickup.setGoal(Kickup.Goal.IDLING);
		}).andThen(Commands.run(() -> {
			leftTurret.setGoal(Turret.Goal.SHOOTING);
			rightTurret.setGoal(Turret.Goal.SHOOTING);
			applyShootCycleGoals(Goal.SHOOTING, shootTimer.hasElapsed(0.5));
		}, leftTurret, rightTurret, kickup, intake).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
			kickup.setGoal(Kickup.Goal.IDLING);
			intake.setGoal(Intake.Goal.INTAKE_OUTER_IDLE);
			shootTimer.stop();
		})))).withName("Shoot Turrets with Intake In");
	}

	private Command buildShootTurretsCommand() {
		return Commands.runOnce(() -> {
			shootTimer.restart();
			kickup.setGoal(Kickup.Goal.IDLING);
		}).andThen(Commands.run(() -> {
			leftTurret.setGoal(Turret.Goal.SHOOTING);
			rightTurret.setGoal(Turret.Goal.SHOOTING);
			applyShootCycleGoals(Goal.INTAKE_GROUND_SHOOT, shootTimer.hasElapsed(0.5));
		}, leftTurret, rightTurret, kickup, intake).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.AIMING);
			rightTurret.setGoal(Turret.Goal.AIMING);
			intake.setGoal(Goal.INTAKE_OUTER_IDLE);
			kickup.setGoal(Kickup.Goal.IDLING);
			shootTimer.stop();
		}));
	}

	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.get();
	}

	public String getAutoName() {
		return autoChooser.getSendableChooser().getSelected();
	}

	/**
	 * For SIMULATION ONLY, return the estimated current draw of the robot.
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {

		return new double[] { Math.min(drivetrainS.getCurrent(), 200), /* hang.getCurrent(), */ intake.getCurrent(),
				leftTurret.getCurrent(), rightTurret.getCurrent() };
		// superStructure.getCurrent() };
	}

	private static void addNTCommands() {
		// Setup all SystemChecks
		for (SubsystemChecker subsystem : getAllSubsystems()) {
			subsystem.setupSystemCheck();
		}
		SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
	}

	/**
	 * RUN EACH system's test command. Does NOT run any checks on vision.
	 * 
	 * @return a command with all of them in a sequence.
	 */
	public static Command allSystemsCheck() {
		return Commands.sequence(
				drivetrainS.getRunnableSystemCheckCommand(),
				visionS.getSystemCheckCommand(),
				// leds.getSystemCheckCommand(),
				// hang.getSystemCheckCommand(),
				intake.getSystemCheckCommand(),
				leftTurret.getSystemCheckCommand(),
				rightTurret.getSystemCheckCommand(),
				kickup.getSystemCheckCommand());

	}

	public static HashMap<String, Double> combineMaps(
			List<HashMap<String, Double>> maps) {
		HashMap<String, Double> combinedMap = new HashMap<>();
		// Iterate over the list of maps
		for (HashMap<String, Double> map : maps) {
			combinedMap.putAll(map);
		}
		return combinedMap;
	}

	public static HashMap<String, Double> getAllTemps() {
		// List of HashMaps
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(), visionS.getTemps(), /* hang.getTemps(), */
				intake.getTemps(), leftTurret.getTemps(), rightTurret.getTemps(), kickup.getTemps());
		// Combine all maps
		HashMap<String, Double> combinedMap = combineMaps(maps);
		return combinedMap;
	}

	/**
	 * Checks EACH system's status (DOES NOT RUN THE TESTS)
	 * 
	 * @return true if ALL systems were good.
	 */
	public static boolean allSystemsOK() {
		return drivetrainS.getTrueSystemStatus() == SubsystemChecker.SystemStatus.OK
				// && leds.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& visionS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				// && hang.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& intake.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& leftTurret.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& rightTurret.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& kickup.getSystemStatus() == SubsystemChecker.SystemStatus.OK;

	}

	public static Collection<ParentDevice> getOrchestraDevices() {

		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		// devices.addAll(hang.getOrchestraDevices());
		devices.addAll(intake.getOrchestraDevices());
		devices.addAll(kickup.getOrchestraDevices());
		devices.addAll(leftTurret.getOrchestraDevices());
		devices.addAll(rightTurret.getOrchestraDevices());
		return devices;
	}

	public static SubsystemChecker[] getAllSubsystems() {

		SubsystemChecker[] subsystems = new SubsystemChecker[6];
		switch (DriveConstants.driveType) {
			case SWERVE:
				subsystems[0] = (Swerve) drivetrainS;
				break;
			case MECANUM:
				subsystems[0] = (Mecanum) drivetrainS;
				break;
			case TANK:
				subsystems[0] = (Tank) drivetrainS;
				break;
		}
		subsystems[1] = visionS;
		// subsystems[2] = hang;
		subsystems[2] = intake;
		subsystems[3] = leftTurret;
		subsystems[4] = rightTurret;
		subsystems[5] = kickup;
		return subsystems;
	}

	public static void updateSimulationWorld() {
		if (fieldSimulation != null)
			fieldSimulation.updateSimulationWorld();
	}
}
