package frc.robot.utils.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ObjectMapper;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.RequiredArgsConstructor;
import frc.robot.utils.GeomUtil.ApproachDirection;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningConstants;

public class VisionConstants {
	public static final FieldType fieldType = FieldType.ANDYMARK;

	public enum AITargets {
		//Make SURE these are in ORDER of the actual classID.
		FUEL

	}
	public static final long[] objLowerHSV = {0,0,100};
	public static final long[] objUpperHSV = {180,45,210};
	// Command specific constants // Aim To Pose
	public static final ApproachDirection aimToPoseApproachDirection = ApproachDirection.FRONT, // Drive And Aim At Pose public static final ApproachDirection
			driveAndAimAtPoseApproachDirection = ApproachDirection.FRONT; // Drive To AI
	public static final ApproachDirection driveToAITargetApproachDirection = ApproachDirection.FRONT;

	public static class FieldConstants {
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		public static double[] aprilTagOffsets = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 }; //0 -1, trust.
	}

	public static final boolean debug = true;

	public static final double ambiguityThreshold = 0.4;
	public static final double objDetectConfidenceThreshold = .6;
	public static final double maxZError = 0.75;
	public static final double maxObjZError = 0.25;
	public static final double maxYawError = 15.0;
	public static final double linearStdDevBaseline = 0.005;
	public static final double angularStdDevBaseline = 0.04;

	public static final double limeLightAngleOffsetDegrees = -40.0;
	public static final double limelightLensHeightoffFloorInches = 22.5;
	public static final String limelightName = "limelight-swerve";

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingDistance = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingDistance",
			Units.feetToMeters(4),
			TuningConstants.isTuningVision);

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingAngle = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingAngle",
			Units.degreesToRadians(3),
			TuningConstants.isTuningVision);

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingTimeout = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingTimeout", 1, TuningConstants.isTuningVision);

	public static final double maxStaleReadingXMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingYMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingRotation = Units.degreesToRadians(2);

	/**
	 * ["0x01210000 / 3", "0x01230000 / 4", "0x02211000 / 5", "0x02213000 / 6"]	
	 * ["SPCA2630 PC Camera:usb_05c8_0a00_002_007", "SPCA2630 PC Camera:usb_05c8_0a00_002_005", "SPCA2630 PC Camera:usb_05c8_0a00_001_004", "SPCA2630 PC Camera:usb_05c8_0a00_001_003"]
	 */
	public static final CameraConfig[] cameras = new CameraConfig[] {
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									Units.inchesToMeters(0), //centered on robot
									Units.inchesToMeters(0),
									Units.inchesToMeters(0), //it changes, we don't know
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(30.237350),
											Math.toRadians(0)))) //we don't know, but it faces forward so we assume no yaw/pitch
					.id("Microsoft® LifeCam HD-3000:usb_045e_0810_001_001")
					.location("0x01100000 / 1")  
					.width(1600)//1600
					.height(1304)//1304
					.exposure(150)
					.saturation(50)
					.hue(0)
					.whiteBalance(4000)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
								-.317,
								-.263,
								.205,
								new Rotation3d(
										Math.toRadians(0.0),
										Math.toRadians(-25),
										Math.toRadians(160))))
										.id("SPCA2630 PC Camera:usb_05c8_0a00_002_005")
										.location("0x01230000 / 4")
					.width(1600)
					.height(1304)
					.exposure(150)
					.saturation(50)
					.hue(0)
					.whiteBalance(4000)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
					CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									-.317,
									.263,
									.205,
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(-25),
											Math.toRadians(-160))))
					.id("SPCA2630 PC Camera:usb_05c8_0a00_001_004")
					.location("0x02211000 / 5")
					.width(1600)
					.height(1304)
					.exposure(150)
					.saturation(50)
					.hue(0)
					.whiteBalance(4000)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
	};

	@Builder
	@NoArgsConstructor
	@AllArgsConstructor
	@Getter
	public static class CameraConfig {
		private Supplier<Pose3d> pose;
		private String id;
		private String location;
		private int width;
		private int height;
		private int autoExposure;
		private int autoWhiteBalance;
		private double exposure;
		private int saturation;
		private int hue;
		private int whiteBalance;
		private double gain;
		private double denoise;
	}
	//Transforms for alternative functions (like aiming)

public static final double aprilTagWidth = Units.inchesToMeters(6.50);
public static final boolean bumperDetection = false;
@RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
	OFFSEASON("offseason"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
	HOME("2026-home"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p =
                  Constants.currentMode == Mode.SIM
                      ? Path.of(
                          "src",
                          "main",
                          "deploy",
                          "apriltags",
                          fieldType.getJsonFolder(),
                          "2026-sim" + ".json")
                      : Path.of(
                          Filesystem.getDeployDirectory().getPath(),
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}