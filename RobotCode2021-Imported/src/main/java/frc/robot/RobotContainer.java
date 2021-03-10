/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.*;
import frc.robot.commands.autos.Slalom;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public static SwerveDrive swerveDrive = new SwerveDrive();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    public final static Shooter shooter = new Shooter();
    private final Climb climb = new Climb();
    private final PhotonCamera camera = new PhotonCamera("TurretCamera");
    private final Vision vision = new Vision(camera);
    private final Turret turret = new Turret(swerveDrive);

    // The driver's controller
    public static Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick m_operatorController = new Joystick(1);

//     public static final GenericHID.RumbleType kLeftRumble = 1;


    public static ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);

    // Network Tables for Vision
    public NetworkTableEntry yaw;
    public NetworkTableEntry isDriverMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
       
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        swerveDrive.setDefaultCommand(new DefaultDrive(swerveDrive, m_driverController, false));
        conveyor.setDefaultCommand(new AutoIndexConveyor(conveyor));
        intake.setDefaultCommand(new RunIntake(intake, m_operatorController));
        turret.setDefaultCommand(new SpinTurret(turret, false, 0));
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton butA = new JoystickButton(m_operatorController, 1);
        JoystickButton butB = new JoystickButton(m_operatorController, 2); 
        JoystickButton butY = new JoystickButton(m_operatorController, 3);
        JoystickButton butpress = new JoystickButton(m_driverController, 8); 
        JoystickButton butXd = new JoystickButton(m_driverController, 8);       
        JoystickButton rBump = new JoystickButton(m_operatorController, 6);
        JoystickButton lBump = new JoystickButton(m_operatorController, 5);
        JoystickButton lAnal = new JoystickButton(m_operatorController, 9);
        JoystickButton rAnal = new JoystickButton(m_operatorController, 10);
        JoystickButton gyro = new JoystickButton(m_driverController, 7);
        

        // A button
        butA.whileHeld(new ExtendIntake(intake, m_operatorController));
        butA.whenReleased(new RetractIntake(intake));

        
        
        // right bumper
        rBump.whileHeld(new RunShooter(shooter));
        rBump.whenReleased(new StopShooter(shooter));
        
        // left analog center
        lAnal.whileHeld(new MoveHood(shooter, -1));
        

        // right analog center
        rAnal.whileHeld(new MoveHood(shooter, 1));
        
        // right bumper
        rBump.whileHeld(new FeedShooter(conveyor, shooter));
        rBump.whenReleased(new AutoIndexConveyor(conveyor));
        
        // left bumper
        lBump.whileHeld(new ControlConveyor(conveyor));
        lBump.whenReleased(new AutoIndexConveyor(conveyor));
        
        // B button
        butB.whileHeld(new SpinTurret(turret, true, 0.25));
        butB.whenReleased(new SpinTurret(turret, true, 0));
        
        // Y button
        butY.whileHeld(new SpinTurret(turret, true, -0.25));
        butY.whenReleased(new SpinTurret(turret, true, 0));

        // driver X button
        butXd.whileHeld(new DefaultDrive(swerveDrive, m_driverController, true));

        gyro.whenPressed(new InstantCommand(swerveDrive::zeroHeading));

        //new JoystickButton(m_operatorController, 4).whenPressed(new RunCommand(() -> conveyor.manualControl(-), conveyor))
        //        .whenReleased(new RunCommand(conveyor::autoIndex, conveyor));
        // should be start button for camera to find target idk what number is so fix it
        // new JoystickButton(m_operatorController, 7).whenHeld(new InstantCommand(turret::visionTurret, turret));
        
}

public static String getCoords() {
    return (swerveDrive.getPose().getX() + " " + swerveDrive.getPose().getY());
}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(SwerveDriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(0, 3, new Rotation2d(0)), config);

        Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new  Rotation2d(-Math.PI / 2)), List.of(
                // keep in
                new Translation2d(0, -1.2), // take out
                new Translation2d(1.1, -1.9),
                new Translation2d(1.88, -1.9),
             
             
                //new Translation2d(1.88, -3.5),
                //new Translation2d(1.88, -4),
             // keep in
                new Translation2d(1.88, -5.5),
                new Translation2d(1.88, -6.25),
                new Translation2d(0.3, -6.31),
                new Translation2d(0.3, -7.9)
      
      
        // previously commented out
                //new Translation2d(1.324, -7.9),
                //new Translation2d(1.724, -7.9),
                // new Translation2d(1.6, -7.75),
                // new Translation2d(1.6, -6.55),
                // new Translation2d(0, -6.3)
                //new Translation2d(1.78, -7.75)
            ), 

           new Pose2d(1.88, -7.75, new Rotation2d(-Math.PI / 2)), config);
        //    new Pose2d(0, -7, new Rotation2d(-Math.PI / 2)), config);



        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(Slalom.getTrajectory(),
                (-Math.PI / 2), swerveDrive::getPose, // Functional interface to feed supplier
                SwerveDriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), theta,

                swerveDrive::setModuleStates,

                swerveDrive

        );

        // Command shootCommand = new Command(() -> shooter.runHood(.5), shooter)
        //                         .andThen(shooter::runShooter, shooter)
        //                         .andThen(new RunCommand(() -> conveyor.feedShooter(0.75, shooter.atSpeed()), conveyor))
        //                         .withTimeout(15).andThen(new InstantCommand(shooter::stopShooter, shooter));

        

        // return swerveControllerCommand1;

        return swerveControllerCommand1;
    }

}
