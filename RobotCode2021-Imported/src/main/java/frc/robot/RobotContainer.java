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
    private final Turret turret = new Turret();
    // The driver's controller
    public static Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick m_operatorController = new Joystick(1);



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
        swerveDrive.setDefaultCommand(new DefaultDrive(swerveDrive, vision, m_driverController, 1));
        conveyor.setDefaultCommand(new AutoIndexConveyor(conveyor));
        intake.setDefaultCommand(new RunIntake(intake, m_operatorController));
        turret.setDefaultCommand(new SpinTurret(turret, swerveDrive, vision, 2, 0));

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
        JoystickButton butX = new JoystickButton(m_operatorController, 3);
        
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

        // GalacticA galA = new GalacticA(swerveDrive, theta);
    
        SmartDashboard.putData(Scheduler.getInstance());
        // SmartDashboard.putData("GalacticA", galA);


        BouncePath Bounce = new BouncePath(swerveDrive, theta);
    
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("Bounce Path", Bounce);

        
        SlalomPath1 Slalom = new SlalomPath1(swerveDrive, theta);
    
        SmartDashboard.putData(Scheduler.getInstance());
        SmartDashboard.putData("Slalom Path", Slalom);

        SideStep SideStep = new SideStep(swerveDrive, theta);
        butAd.whenPressed(SideStep);


        // A button
        butA.whileHeld(new ExtendIntake(intake, m_operatorController));
        butA.whenReleased(new RetractIntake(intake));

        
        
        // right bumper
        rBump.whileHeld(new RunShooter(shooter));
        rBump.whenReleased(new StopShooter(shooter));
        rBump.whileHeld(new FeedShooter(conveyor, shooter));
        rBump.whenReleased(new AutoIndexConveyor(conveyor));

        // left bumper
        lBump.whileHeld(new ControlConveyor(conveyor));
        lBump.whenReleased(new AutoIndexConveyor(conveyor));

        // left analog center
        lAnal.whileHeld(new MoveHood(shooter, -1));   

        // right analog center
        rAnal.whileHeld(new MoveHood(shooter, 1));
        
        
        
        
        // B button
        butB.whileHeld(new SpinTurret(turret, swerveDrive, vision, 1, 0.25));
        
        // Y button
        butX.whileHeld(new SpinTurret(turret, swerveDrive, vision, 1, -0.25));

        // driver X button - slow
        butXd.whileHeld(new DefaultDrive(swerveDrive, vision, m_driverController, 0.35));
        butAd.whenPressed(new InstantCommand(swerveDrive::zeroWheels));

        butBd.whenPressed(new SideStep(swerveDrive, theta));

        
        turbo.whileHeld(new DefaultDrive(swerveDrive, vision, m_driverController, 2));

        gyro.whenPressed(new InstantCommand(swerveDrive::zeroHeading));

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
        // GalacticA galA = new GalacticA(swerveDrive, theta);
        BarrelPath bar = new BarrelPath(swerveDrive, theta);

        return bar;

        return swerveControllerCommand1;
    }

}
