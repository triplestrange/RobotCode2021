// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Auto;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.command.InstantCommand;

// import java.util.List;

// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.SwerveDriveConstants;
// import frc.robot.subsystems.*;

// public class GalacticA extends CommandGroup {
//         /** Add your docs here. */
//         public GalacticA(SwerveDrive swerveDrive, ProfiledPIDController theta) {

//                 final RobotContainer m_robotContainer;
//                 final Intake intake;

//                 // Create config for trajectory
//                 TrajectoryConfig config = new TrajectoryConfig(2.1,
//                                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                                                 // Add kinematics to ensure max speed is actually obeyed
//                                                 // .setKinematics(SwerveDriveConstants.kDriveKinematics)
//                                                 .setEndVelocity(1.5);

//                 Trajectory trajectory = TrajectoryGenerator
//                                 .generateTrajectory(new Pose2d(0, 0, new Rotation2d(-Math.PI / 2)), List.of(

//                                 ),
//                                                 // direction robot moves
//                                                 new Pose2d(0, -3, new Rotation2d(Math.PI / 2)), config);

//                 SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(trajectory, (0),
//                                 swerveDrive::getPose, // Functional interface to feed supplier
//                                 SwerveDriveConstants.kDriveKinematics,

//                                 // Position controllers
//                                 new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
//                                 new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

//                                 swerveDrive::setModuleStates,

//                                 swerveDrive

//                 );

//                 addSequential(swerveControllerCommand1);
               

//         }

// }
