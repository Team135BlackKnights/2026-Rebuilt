package frc.robot.commands.auto;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;


//Go to hub center, at x of 10
public class AutoIntake extends Command{
    private boolean isFinished = false;
    @Override
    public void initialize() {    
        isFinished = false;
        //for now, just go using Pathfinder
        CommandScheduler.getInstance().schedule(PathFinder.goToAutoPilotPose(RobotContainer.pathFinder, new APTarget(new Pose2d(4.4,6,new Rotation2d())), RobotContainer.drivetrainS, () ->DriveConstants.pathConstraints, 1.0, .2));
    }
    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
     @Override
    public void end(boolean interrupted) {
        isFinished = true;
    }
}
