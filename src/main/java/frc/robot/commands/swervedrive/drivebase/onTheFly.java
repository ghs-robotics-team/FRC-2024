// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class onTheFly extends CommandBase {
    private final SwerveSubsystem drivebase;
    private final int id;
    private final Rotation2d rotation;
    private final Rotation2d holonomicRotation;
    private final Translation2d offset;

    public onTheFly(SwerveSubsystem drivebase, int id, Rotation2d rotation,
                                  Rotation2d holonomicRotation, Translation2d offset) {
        this.drivebase = drivebase;
        this.id = id;
        this.rotation = rotation;
        this.holonomicRotation = holonomicRotation;
        this.offset = offset;

        addRequirements(this.drivebase);
    }

    @Override
    public void initialize() {
        //drivebase.addFakeVisionReading();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Drivebase mode", "On The Fly");
        Pose2d currentPose = drivebase.getPose();
        Rotation2d heading = new Rotation2d(drivebase.getFieldVelocity().vxMetersPerSecond, drivebase.getFieldVelocity().vyMetersPerSecond);

        CommandBase command = Autos.driveToAprilTag(drivebase, id, rotation, holonomicRotation, offset, currentPose, heading);
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}