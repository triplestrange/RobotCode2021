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

public class SlalomPath1 extends CommandGroup {
  /** Add your docs here. */
  public SlalomPath1(SwerveDrive swerveDrive, ProfiledPIDController theta) {

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(2.2,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                      .setEndVelocity(2);

      TrajectoryConfig config1 = new TrajectoryConfig(2.2,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      // .setKinematics(SwerveDriveConstants.kDriveKinematics)
                      .setStartVelocity(2);

                    
    
    Trajectory traject = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new  Rotation2d(0)), List.of(
          //start s-shape
          new Translation2d(-.25, -1.4),
          new Translation2d(1.1, -1.7),
          new Translation2d(1.88, -2),


          new Translation2d(1.9, -4),
          new Translation2d(1.88, -6.5),
          new Translation2d(-.3, -6.5),
          new Translation2d(-.3, -8.35),
          new Translation2d(1.3, -8.25),
          new Translation2d(1.88, -6.5)

          // make t hsi point go a litte bit up (hitting cube)
          // new Translation2d(-0.5, -6.5)
      ), 
                           //direction robot moves
     new Pose2d(-.5, -5.5, new Rotation2d(Math.PI / 2)), config);


     Trajectory traject1 = TrajectoryGenerator.generateTrajectory(
                                 //direction robot moves
      new Pose2d(-.5, -5.5, new  Rotation2d(Math.PI / 2)), List.of(
          new Translation2d(-.5, -1.55),
          new Translation2d(1.5, -1.65)
      ), 
      
     new Pose2d(2.1, 0, new Rotation2d(Math.PI / 2)), config1);
  //    new Pose2d(0, -7, new Rotation2d(-Math.PI / 2)), config);

  // String  trajectoryJSON = "../paths/Slalom.wpilib.json";
  // Trajectory trajectory = new Trajectory();
  // try {
  //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  // } catch (IOException ex) {
  //     DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
  // }


  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(traject,
          (0), swerveDrive::getPose, // Functional interface to feed supplier
          SwerveDriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
          new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

          swerveDrive::setModuleStates,

          swerveDrive

  );
  SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(traject1,
  (0), swerveDrive::getPose, // Functional interface to feed supplier
  SwerveDriveConstants.kDriveKinematics,

  // Position controllers
  new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
  new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

  swerveDrive::setModuleStates,

  swerveDrive

);

  // Command shootCommand = new Command(() -> shooter.runHood(.5), shooter)
  //                         .andThen(shooter::runShooter, shooter)
  //                         .andThen(new RunCommand(() -> conveyor.feedShooter(0.75, shooter.atSpeed()), conveyor))
  //                         .withTimeout(15).andThen(new InstantCommand(shooter::stopShooter, shooter));

  addSequential(swerveControllerCommand1);
  addSequential(swerveControllerCommand2);
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    
  }
}
