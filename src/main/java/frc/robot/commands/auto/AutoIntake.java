package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.drive.vision.AimToObject;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Goal;
import frc.robot.subsystems.vision.VisionIO.CameraID;

public class AutoIntake extends ParallelCommandGroup {

  public AutoIntake(
      DrivetrainS drive,
      Intake intake,
     CameraID cam
  ) {
    addCommands(
        new AimToObject(drive, cam, 0, 0),
        Commands.run(() -> intake.setGoal(Goal.INTAKE_GROUND), intake)
            .finallyDo(() -> intake.setGoal(Goal.INTAKE_OUTER_IDLE))
    );

    setName("AutoIntake");
  }
}
