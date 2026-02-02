// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OrchestraC;
import frc.robot.commands.StaticCharacterization;
import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.subsystems.SubsystemChecker;
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
import frc.robot.subsystems.Turret.kickup.KickupIOKrakenFOC;
import frc.robot.subsystems.Turret.kickup.KickupIOSim;
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
import frc.robot.subsystems.intake.Indexer.Indexer;
import frc.robot.subsystems.intake.Indexer.IndexerIO;
import frc.robot.subsystems.intake.Indexer.IndexerIOKrakenFOC;
import frc.robot.subsystems.intake.Indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Indexer.IndexerIOSparkBase;
import frc.robot.subsystems.intake.Intake.Goal;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.intake.arm.ArmIOSim;
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
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber.Climber;
import frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber.ClimberIO;
import frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber.ClimberIOKrakenFOC;
import frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber.ClimberIOSim;

import frc.robot.utils.DriverStationHID;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Rebuilt2026FieldSimulation;
import frc.robot.utils.leds.LEDConstants.ImageStates;
import frc.robot.utils.robotToggles.Toggles;
import frc.robot.utils.robotToggles.TogglesIO;
import frc.robot.utils.robotToggles.TogglesIOHardware;
import frc.robot.utils.robotToggles.TogglesIONetworkTables;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

import frc.robot.utils.Touchboard.PosePlotterUtil;
import frc.robot.utils.Touchboard.JukeboxUtil;
import frc.robot.utils.Touchboard.PosePlotterUtil.CommandPair;
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
	private static final LEDs leds = new LEDs();
	public static Vision visionS;
	public static Toggles toggles;
	public static LocalADStarAK pathFinder = new LocalADStarAK();
	public static Climber climber;
	public static Turret leftTurret;
	public static Turret rightTurret;
	public static Intake intake;
	private final LoggedDashboardChooser<Command> autoChooser;
	public static final LoggableTunedNumber humanPlayerWaitTime = new LoggableTunedNumber(
			"AutoToggles/HumanPlayerWaitTime", .425, TuningConstants.isTuningMacros);
	// [Map<String,>,]
	public static CommandXboxController driveController = new CommandXboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static DriverStationHID dsHIDHandler = new DriverStationHID(2);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	public static double xSpeed = 0;
	public static double ySpeed = 0;
	@AutoLogOutput(key = "SuperStructure/ScorePosition")

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
	public static int currentTest = 0;
	public static String piConnection = "DISCONNECTED";
	@AutoLogOutput(key = "RobotState/currentPath")
	public static String currentPath = "";
	public static Field2d field = new Field2d();
	public static boolean userDrive = true;
	public static boolean withinLineTolerance = false;
	@AutoLogOutput(key = "RobotState/miloMad")
	public static boolean miloMad = false;
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
		List<Pair<String, CommandPair>> autoCommands = new ArrayList<>();
		String[] auto = PosePlotterUtil.getAutoString().split("_");
		if (auto.length < 2) {
			auto = new String[] { "-120", "5.542", "NA" };
		}
		double x = Double.parseDouble(auto[1]);
		double y = 7.02;
		double theta = Units.degreesToRadians(Double.parseDouble(auto[0]));
		Pose2d startingPose = new Pose2d(x, y, new Rotation2d(theta));
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
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "FrontRightCam", 0,
								VisionConstants.cameras[0]),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "FrontLeftCam", 1,
								VisionConstants.cameras[1]),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackRightCam", 2,
								VisionConstants.cameras[2]),
						new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackLeftCam", 3,
								VisionConstants.cameras[3]));
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
				switch (SimpleMechanismConstants.Climber.motorType) {
					case CTRE_ON_RIO:
					case CTRE_ON_CANIVORE:
						climber = new Climber(new ClimberIOKrakenFOC());
						break;
					default:
						throw new IllegalArgumentException(
								"Unknown implementation type for climber (REV NOT SUPPORTED!), please check SimpleMechanismConstants.java!");
				}
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
								true,
								IntakeConstants.intakeReductionToIndexerRollers));
				intake = new Intake(armIO, indexer);
				// Left Turret
				AzimuthIO azimuthIOLeftTurret = new AzimuthIOKrakenFOC(Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftAzimuthID,
						AdvancedMechanismConstants.Turret.leftAzimuthBigEncoderID,
						AdvancedMechanismConstants.Turret.leftAzimuthSmallEncoderID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitAzimuth,
						AdvancedMechanismConstants.Turret.minTurretAngle,
						AdvancedMechanismConstants.Turret.maxTurretAngle);

				FlywheelIO flywheelIOLeftTurret = new FlywheelIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftFlywheelID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitFlywheel,
						AdvancedMechanismConstants.Turret.flywheelRatio);
				HoodIO hoodIOLeftTurret = new HoodIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.leftHoodID,
						AdvancedMechanismConstants.Turret.leftHoodEncoderID,
						AdvancedMechanismConstants.Turret.leftName,
						AdvancedMechanismConstants.Turret.currentLimitHood,
						AdvancedMechanismConstants.Turret.hoodMotorToHoodEncoderRatio,
						AdvancedMechanismConstants.Turret.hoodEncoderToHoodArmRatio,
						AdvancedMechanismConstants.Turret.minHoodAngle,
						AdvancedMechanismConstants.Turret.maxHoodAngle);
				Kickup kickupLeftTurret = new Kickup(new KickupIOKrakenFOC(AdvancedMechanismConstants.Turret.leftKickupID, Robot.rioCanBus, "LeftKickup", AdvancedMechanismConstants.Turret.currentLimitKickup, AdvancedMechanismConstants.Turret.invertKickup, true, AdvancedMechanismConstants.Turret.kickerRatio));
				leftTurret = new Turret(azimuthIOLeftTurret, flywheelIOLeftTurret, hoodIOLeftTurret,kickupLeftTurret,
						AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter, "LeftTurret");
				// Right Turret
				AzimuthIO azimuthIORightTurret = new AzimuthIOKrakenFOC(Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightAzimuthID,
						AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID,
						AdvancedMechanismConstants.Turret.rightAzimuthSmallEncoderID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitAzimuth,
						AdvancedMechanismConstants.Turret.minTurretAngle,
						AdvancedMechanismConstants.Turret.maxTurretAngle);
				FlywheelIO flywheelIORightTurret = new FlywheelIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightFlywheelID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitFlywheel,
						AdvancedMechanismConstants.Turret.flywheelRatio);
				HoodIO hoodIORightTurret = new HoodIOKrakenFOC(
						Robot.rioCanBus,
						AdvancedMechanismConstants.Turret.rightHoodID,
						AdvancedMechanismConstants.Turret.rightHoodEncoderID,
						AdvancedMechanismConstants.Turret.rightName,
						AdvancedMechanismConstants.Turret.currentLimitHood,
						AdvancedMechanismConstants.Turret.hoodMotorToHoodEncoderRatio,
						AdvancedMechanismConstants.Turret.hoodEncoderToHoodArmRatio,
						AdvancedMechanismConstants.Turret.minHoodAngle,
						AdvancedMechanismConstants.Turret.maxHoodAngle);
				Kickup kickupRightTurret = new Kickup(new KickupIOKrakenFOC(AdvancedMechanismConstants.Turret.rightKickupID, Robot.rioCanBus, "RightKickup", AdvancedMechanismConstants.Turret.currentLimitKickup, AdvancedMechanismConstants.Turret.invertKickup, true, AdvancedMechanismConstants.Turret.kickerRatio));
				rightTurret = new Turret(azimuthIORightTurret, flywheelIORightTurret, hoodIORightTurret,kickupRightTurret,
						AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter, "RightTurret");

				autoCommands.addAll(Arrays.asList(

				// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
				// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
				// new Pair<String, Command>("BranchGrabbingGamePiece",
				// new BranchAuto("Shoot",
				// new Pose2d(7.4, 5.8, new Rotation2d()), 4))
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));
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
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "FrontRightCam",0,
				 * VisionConstants.cameras[0]),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "FrontLeftCam",1,
				 * VisionConstants.cameras[1]),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackRightCam",2,
				 * VisionConstants.cameras[2]),
				 * new VisionIOSouthmoon(() -> getSelectedAprilTagLayout(), "BackLeftCam",3,
				 * VisionConstants.cameras[3]));
				 */
				visionS = new Vision(() -> getSelectedAprilTagLayout(),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "FrontRightCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[0].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "FrontLeftCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[1].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "BackRightCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[2].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(() -> getSelectedAprilTagLayout(), "BackLeftCam",
								GeomUtil.poseToTransform3d(VisionConstants.cameras[3].getPose().get()),
								() -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()));
				toggles = new Toggles(new TogglesIONetworkTables());
				System.out.println("SIM SETUP DONE!");
				climber = new Climber(new ClimberIOSim());
				intake = new Intake(new ArmIOSim(),new Indexer(new IndexerIOSim(DCMotor.getKrakenX44Foc(1), "Indexer",
						IntakeConstants.intakeReductionToIndexerRollers, IntakeConstants.intakeMOI)));
						
				leftTurret = new Turret(new AzimuthIOSim(-Math.PI, Math.PI), new FlywheelIOSim(), new HoodIOSim(),
					new Kickup(new KickupIOSim(AdvancedMechanismConstants.Turret.kickupMotor, "Kickup", AdvancedMechanismConstants.Turret.kickerRatio, AdvancedMechanismConstants.Turret.kickupMOI)),
						AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter, "LeftTurret");
				rightTurret = new Turret(new AzimuthIOSim(-Math.PI, Math.PI), new FlywheelIOSim(), new HoodIOSim(),
						new Kickup(new KickupIOSim(AdvancedMechanismConstants.Turret.kickupMotor, "Kickup", AdvancedMechanismConstants.Turret.kickerRatio, AdvancedMechanismConstants.Turret.kickupMOI)),
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
						}, new VisionIO() {
						}); // MUST be same number of cameras as in real robot
				toggles = new Toggles(new TogglesIO() {
				});
				climber = new Climber(new ClimberIO() {
				});
				intake = new Intake(new ArmIO() {
				}, new Indexer(new IndexerIO() {
				}));
				leftTurret = new Turret(new AzimuthIO() {
				}, new FlywheelIO() {
				}, new HoodIO() {
				}, new Kickup(new KickupIO() {
				}),AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter, "LeftTurret");
				rightTurret = new Turret(new AzimuthIO() {
				}, new FlywheelIO() {
				}, new HoodIO() {
				},
				 new Kickup(new KickupIO() {
				 }), AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter, "RightTurret");

		}

		drivetrainS.resetPose(GeomUtil.apply(startingPose, false));
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		Pathfinding.setPathfinder(pathFinder);
		// algaeScorer.setDefaultCommand(new AlgaeScorerC(algaeScorer));
		// superStructureNotifier = new Notifier(superStructure::periodic);
		// superStructureNotifier.startPeriodic(.01);
		// Add all the auto commands to the auto builder

		// Make sure to watch your flipped poses. Our custom DriveToPose and all of
		// those do NOT auto flip for red.
		autoCommands.addAll(Arrays.asList(
		/*
		 * new Pair<String, CommandPair>("RT", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(FieldConstants.CoralStation.blueRightTopFace, false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 0, .5,.1),
		 * Set.of(drivetrainS))),
		 * new Pair<String, CommandPair>("RM", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(FieldConstants.CoralStation.blueRightCenterFace, false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 0, .5,.1),
		 * Set.of(drivetrainS))),
		 * new Pair<String, CommandPair>("RB", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(FieldConstants.CoralStation.blueRightBottomFace, false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 0, .5,.1),
		 * Set.of(drivetrainS))),
		 * new Pair<String, CommandPair>("10", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(new Pose2d(4,2.82,new Rotation2d(Math.PI/3)), false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 1, .5,.05),
		 * Set.of(drivetrainS))),
		 * new Pair<String, CommandPair>("11", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(new Pose2d(3.693,3.01,new Rotation2d(Math.PI/3)), false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 1, .5,.05),
		 * Set.of(drivetrainS))),
		 * new Pair<String, CommandPair>("12", // Example Drive to the right top face of
		 * the coral station
		 * new CommandPair(
		 * (Supplier<Command>) () -> PathFinder.goToPose(
		 * GeomUtil.apply(new Pose2d(3.211,3.883,new Rotation2d(0)), false),
		 * () -> DriveConstants.pathConstraints, drivetrainS, false, 1, .5,.05),
		 * Set.of(drivetrainS)))
		 */));
		precalculateAllStartAndEndChoreos();

		for (Pair<String, CommandPair> autoCommand : autoCommands) {
			PosePlotterUtil.addCommandPair(autoCommand.getFirst(), autoCommand.getSecond());
		}

		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}

		CommandScheduler.getInstance().schedule(new PathfindingCommand(
				new Pose2d(15.0, 4.0, Rotation2d.k180deg),
				new PathConstraints(8, 11, 4, 4),
				() -> new Pose2d(1.5, 4, Rotation2d.kZero),
				ChassisSpeeds::new,
				(speeds, feedforwards) -> {
				},
				new PPHolonomicDriveController(
						new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
				DriveConstants.mainConfig)
				.andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
				.ignoringDisable(true)
				.finallyDo(() -> RobotContainer.field.getObject("target pose")
						.setPose(new Pose2d(-50, -50, new Rotation2d()))));
		if (!leds.gifFound(ImageStates.debug)) {
			Logger.recordOutput("LEDS/Main",
					"No images found for " + ImageStates.debug.name());
		}
		if (!AutoBuilder.isConfigured()) {
			throw new RuntimeException(
					"AutoBuilder was not configured before attempting to build an auto chooser");
		}
		JukeboxUtil jukebox = new JukeboxUtil();
		for (ParentDevice device : getOrchestraDevices()) {
			jukebox.addTalon(device);
		}
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		autoChooser.addDefaultOption("DynamicPathing",
				Commands.defer(() -> PosePlotterUtil.getAuto(), Set.of(drivetrainS)));
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
		SmartDashboard.putData(field);
		// Store the last known value of autoChooser.get()

		new Thread(() -> {
			while (true) {
				try {
					// Get the current value from autoChooser
					Command currentAutoValue = autoChooser.get();

					// Check if the value has changed
					if (currentAutoValue != null) {
						if (!currentAutoValue.equals(lastAuto)) {
							// Update the last known value
							lastAuto = currentAutoValue;
							// Run your logic
							try {
								currentAuto = currentAutoValue;
								Logger.recordOutput("RobotState/autoPath",
										PathFinder.parseAutoToPose2dList(currentAutoValue.getName())
												.toArray(Pose2d[]::new));
								field.getObject("path")
										.setPoses(PathFinder.parseAutoToPose2dList(currentAutoValue.getName()));
							} catch (Exception e) {
								System.err.println("NO FOUND PATH FOR DESIRED AUTO!! Dyanmic?");
								field.getObject("path").setPoses(
										new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
												new Pose2d(-50.2, -50, new Rotation2d())
										});
							}
						}
					}

					// Sleep for a short duration to prevent excessive CPU usage
					Thread.sleep(250); // Adjust the interval as necessary
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					System.err.println("Polling thread interrupted");
					break;
				}
			}
		}).start();

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
		Trigger inScoreArea = new Trigger(() -> drivetrainS.getPose().getX() < 5.0);

		var aimHubBoth = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
		}, leftTurret, rightTurret);

		var targetHubBoth = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.HUB_TOP_CENTER);
		}, leftTurret, rightTurret);

		// Opposing trenches (left to left) = each turret shoots its own side
		var targetSplitTrenches = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.LEFT_TRENCH_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.RIGHT_TRENCH_CENTER);
		}, leftTurret, rightTurret);

		var targetBothLeftTrench = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.LEFT_TRENCH_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.LEFT_TRENCH_CENTER);
		}, leftTurret, rightTurret);

		var targetBothRightTrench = Commands.runOnce(() -> {
			leftTurret.setPresetTarget(Turret.PresetTarget.RIGHT_TRENCH_CENTER);
			rightTurret.setPresetTarget(Turret.PresetTarget.RIGHT_TRENCH_CENTER);
		}, leftTurret, rightTurret);

		var idleBoth = Commands.runOnce(() -> {
			leftTurret.setGoal(Turret.Goal.IDLE);
			rightTurret.setGoal(Turret.Goal.IDLE);
		}, leftTurret, rightTurret);
		var shootTurrets = Commands.run(() -> {
			leftTurret.setGoal(Turret.Goal.SHOOTING);
			rightTurret.setGoal(Turret.Goal.SHOOTING);
		}, leftTurret, rightTurret).finallyDo(() -> {
			leftTurret.setGoal(Turret.Goal.IDLE);
			rightTurret.setGoal(Turret.Goal.IDLE);
		});
		//Start of actual bindings
		startButtonDrive
				.onTrue(new InstantCommand(() -> {
					System.out.println("Zeroing Gyro");
					drivetrainS.zeroHeading();
					// drivetrainS.resetPose(GeomUtil.apply(startingPose.get(), false));
				}));
		selectButtonDrive 
				.onTrue(new InstantCommand(() -> {
					System.out.println("Resetting Systems");
					//TODO: More reset stuff
				}));
		leftStickButtonDrive.onTrue(drivetrainS.orientModules(Swerve.getXOrientations()));
		leftStickButtonDrive.onFalse(Commands.runOnce(() -> drivetrainS.stopModules(), drivetrainS));
		rightStickButtonDrive.onTrue(new OrchestraC("megolovania").withName("Play Megolovania"));
		// Climber controls
		aButtonDrive.toggleOnTrue(Commands.none()); // Prepare Climb
		bButtonDrive.toggleOnTrue(Commands.none()); // Climb Sequence
		yButtonDrive.toggleOnTrue(Commands.none()); // Emergency Stop Climb
		//Intake controls
		leftBumperDrive.whileTrue(Commands.run(() -> intake.setGoal(Goal.INTAKE_GROUND)).finallyDo(() ->intake.setGoal(Goal.STOW)));
		rightBumperDrive.onChange(new InstantCommand(() -> {
			DriveConstants.autoIntake = !DriveConstants.autoIntake; // Assist with intake driving.
		}));
		xButtonDrive.whileTrue(Commands.either(Commands.run(() -> intake.setGoal(Goal.JACKHAMMERING_OUT),intake).finallyDo(() ->intake.setGoal(Goal.STOW)), Commands.run(() -> intake.setGoal(Goal.JACKHAMMERING_IN), intake).finallyDo(() ->intake.setGoal(Goal.STOW)), () -> intake.isIntakeDeployed())); // jackhammer
		leftTriggerDriveFull.whileTrue(PathFinder.goToAutoPilotPose(pathFinder,
				new APTarget(GeomUtil.apply(new Pose2d(8, 3.5, Rotation2d.fromDegrees(-45)), false)), drivetrainS,
				() -> DriveConstants.pathConstraints, 1, .02));// make this a "go to our alliance's zone"
		// Turret Controls
		rightTriggerDriveFull.whileTrue(shootTurrets);
		povUp.onTrue(targetHubBoth);
		povDown.onTrue(targetSplitTrenches);
		povLeft.onTrue(targetBothLeftTrench);
		povRight.onTrue(targetBothRightTrench);

		// - If in score area AND not manually holding POV: aim both turrets at hub
		/*inScoreArea.and(manualTurretControl.negate())
				.whileTrue(aimHubBoth);*/

		// - If NOT in score area AND not holding POV: idle both turrets
		inScoreArea.negate().and(manualTurretControl.negate())
				.onTrue(idleBoth);

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
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
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

		return new double[] { Math.min(drivetrainS.getCurrent(), 200), climber.getCurrent(), intake.getCurrent(),
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
				leds.getSystemCheckCommand(),
				climber.getSystemCheckCommand(),
				intake.getSystemCheckCommand(),
				leftTurret.getSystemCheckCommand(),
				rightTurret.getSystemCheckCommand());

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
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(), visionS.getTemps(), climber.getTemps(),
				intake.getTemps(), leftTurret.getTemps(), rightTurret.getTemps());
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
				&& leds.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& visionS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& climber.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& intake.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& leftTurret.getSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& rightTurret.getSystemStatus() == SubsystemChecker.SystemStatus.OK;

	}

	public static Collection<ParentDevice> getOrchestraDevices() {

		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		devices.addAll(climber.getOrchestraDevices());
		devices.addAll(intake.getOrchestraDevices());
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
		subsystems[2] = climber;
		subsystems[3] = intake;
		subsystems[4] = leftTurret;
		subsystems[5] = rightTurret;
		return subsystems;
	}

	public static void updateSimulationWorld() {
		if (fieldSimulation != null)
			fieldSimulation.updateSimulationWorld();
	}
}
