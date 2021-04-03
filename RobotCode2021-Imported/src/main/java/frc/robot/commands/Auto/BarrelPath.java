// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.*;

public class BarrelPath extends CommandGroup {
  /** Add your docs here. */
  public BarrelPath(SwerveDrive swerveDrive, ProfiledPIDController theta) {

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(2.1,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                      .setEndVelocity(2);

//       TrajectoryConfig config1 = new TrajectoryConfig(1,
//               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                       // Add kinematics to ensure max speed is actually obeyed
//                       // .setKinematics(SwerveDriveConstants.kDriveKinematics)
//                       .setStartVelocity(1.5);

                    
    
    Trajectory traject = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
          new Translation2d(0, -3),
          new Translation2d(-1.63, -3.224939),
          new Translation2d(-1.663700, -1.846359),
          new Translation2d(-.180766, -1.846359),
          new Translation2d(-.180766, -5.030811),
          new Translation2d(1.439370, -5.430811),
          new Translation2d(1.439370, -3.995877),
          new Translation2d(-1.827272, -3.995877),
          new Translation2d(-1.827272, -6.816515),
          new Translation2d(0, -6.816515)


      ), 

                           //direction robot moves
     new Pose2d(-0.15, 0, new Rotation2d(Math.PI / 2)), config);


     

  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject,
          (0), swerveDrive::getPose, // Functional interface to feed supplier
          SwerveDriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
          new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

          swerveDrive::setModuleStates,

          swerveDrive

  );
//   SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(traject1,
//   (0), swerveDrive::getPose, // Functional interface to feed supplier
//   SwerveDriveConstants.kDriveKinematics,

//   // Position controllers
//   new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
//   new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

//   swerveDrive::setModuleStates,

//   swerveDrive

// );

  addSequential(swerveControllerCommand1);
//   addSequential(swerveControllerCommand2);
  }
}
