// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.HashMap;
import java.util.List;


public class Autos extends CommandBase
{
  private static AprilTagFieldLayout aprilTagField = null;

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  //Example on the fly path, WIP for limelight integration
  public static CommandBase exampleAuto(SwerveSubsystem swerve)
  {
    PathPlannerTrajectory path;
    Transform2d other = new Transform2d(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(1, 1, Rotation2d.fromDegrees(0)));
      path = PathPlanner.generatePath(
          new PathConstraints(4, 3),
          PathPoint.fromCurrentHolonomicState(swerve.getPose(),swerve.getRobotVelocity()),
          new PathPoint((swerve.getPose().transformBy(other)).getTranslation(), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
          );
    return Commands.sequence(new FollowTrajectory(swerve, path, true));
  }


  public static CommandBase autoBuilderBase(SwerveSubsystem aBuilder, String pathName){
    List<PathPlannerTrajectory> master = PathPlanner.loadPathGroup(pathName, new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakeout", new PrintCommand("Intakeout"));
    eventMap.put("balance", new AutoBalanceCommand(aBuilder));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      aBuilder::getPose,
      aBuilder::resetOdometry,
      new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
      new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
      aBuilder::setChassisSpeeds,
      eventMap,
      false,
      aBuilder
  );
  return Commands.sequence(autoBuilder.fullAuto(master));
  }

  /**
   * Create a {@link FollowTrajectory} command to go to the April Tag from the current position.
   *
   * @param swerve            Swerve drive subsystem.
   * @param id                April Tag ID to go to.
   * @param rotation          Rotation to go to.
   * @param holonomicRotation Holonomic rotation to be at.
   * @param offset            Offset from the April Tag.
   * @return {@link FollowTrajectory} command. May return null if cannot load field.
   */
  public static CommandBase driveToAprilTag(SwerveSubsystem drivebase, int id, Rotation2d rotation, Rotation2d holonomicRotation, Translation2d offset, 
                                            Pose2d currentPose, Rotation2d heading)
  {
    if (aprilTagField == null)
    {
      try
      {
        aprilTagField = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
      } catch (Exception ignored)
      {
        return null;
      }
    }
    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4.2, 6), false,
                                                          new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
                                                          new PathPoint(aprilTagField.getTagPose(id).get()
                                                                                     .getTranslation()
                                                                                     .toTranslation2d().plus(offset),
                                                                        rotation, holonomicRotation));
    return Commands.sequence(new FollowTrajectory(drivebase, path, false));
  }
}
