package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup
{

  public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }
    System.out.println("robot x: " + (drivebase.getPose()).getX());
    System.out.println("robot y: " + (drivebase.getPose()).getY());
    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase)
               );
  }
}
