/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.photonvision.PhotonCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.*;

import frc.robot.commands.Auto.*;

import frc.robot.commands.Auto.BarrelPath;
import frc.robot.commands.Auto.BouncePath;
import frc.robot.commands.Auto.SlalomPath1;

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
    private final PhotonCamera camera1 = new PhotonCamera("OtherCamera");
    private final Vision vision = new Vision(camera);
    private final Vision vision1 = new Vision(camera1);
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
        swerveDrive.setDefaultCommand(new DefaultDrive(swerveDrive, m_driverController, 1));
        conveyor.setDefaultCommand(new AutoIndexConveyor(conveyor));
        intake.setDefaultCommand(new RunIntake(intake, m_operatorController));
        turret.setDefaultCommand(new SpinTurret(turret, false, 0));
       
        // vision.setDefaultCommand(new RunCommand(vision::runVision, vision));

    //     swerveDrive.setDefaultCommand(

    //                             new InstantCommand(() -> swerveDrive.drive(-m_driverController.getRawAxis(1)
    //                                             * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
    //                                             -m_driverController.getRawAxis(0)
    //                                                             * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond,
    //                                             -m_driverController.getRawAxis(4) * (2 * Math.PI), true), swerveDrive));
    }

    /**
     * 
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        //starts with 1
        JoystickButton butA = new JoystickButton(m_operatorController, 1);
        JoystickButton butB = new JoystickButton(m_operatorController, 2); 
        JoystickButton butY = new JoystickButton(m_operatorController, 3);
        
        JoystickButton butXd = new JoystickButton(m_driverController, 3);  
        JoystickButton butAd = new JoystickButton(m_driverController, 1);  
        JoystickButton butBd = new JoystickButton(m_driverController, 2);

        JoystickButton gyro = new JoystickButton(m_driverController, 8);

        JoystickButton rBump = new JoystickButton(m_operatorController, 6);
        JoystickButton lBump = new JoystickButton(m_operatorController, 5);
        JoystickButton lAnal = new JoystickButton(m_operatorController, 9);
        JoystickButton rAnal = new JoystickButton(m_operatorController, 10);

        // B
        JoystickButton turbo = new JoystickButton(m_driverController, 2);
        // JoystickButton ok = new JoystickButton(m_driverController, 7);

        BarrelPath Barrel = new BarrelPath(swerveDrive, theta);
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("Barrel Path", Barrel);

        BouncePath Bounce = new BouncePath(swerveDrive, theta);
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("Bounce Path", Bounce);

        SlalomPath1 Slolam = new SlalomPath1(swerveDrive, theta);
    
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("Slalom Path", Slolam);

        GalacticPathA galA = new GalacticPathA(swerveDrive, intake, theta);
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("GalacticA", galA);

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

        // driver X button - slow
        butXd.whileHeld(new DefaultDrive(swerveDrive, m_driverController, 0.35));
        butAd.whenPressed(new InstantCommand(swerveDrive::zeroWheels));

        butBd.whenPressed(new SideStep(swerveDrive, theta));

        turbo.whileHeld(new DefaultDrive(swerveDrive, m_driverController, 2));

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
    
     public Command getAutonomousCommand(Trajectory trajectory) {
        GalacticA galA = new GalacticA(swerveDrive, theta);

        return galA;
        
    }

}
