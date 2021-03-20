/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultDrive extends Command {
  private final SwerveDrive m_drive;
  private final Joystick m_joystick;
  private  double m_xSpeed, m_ySpeed, m_rot;
  private final boolean m_fieldRelative;
  private double heading;
  private PIDController pid = new PIDController(0.05, 0, 0.01);
  private PIDController visionController = new PIDController(.1, 0, 0);
  private JoystickButton butX;
  private boolean slow;
  private int mode;
  private double multiplier;
  private Vision m_vision;

  /**
   * Creates a new DefaultDrive.
   * 
   * @param subsystem The drive subsystem this command will run on
   * @param driver The joystick to be used for calculations in speed and rotation
   */
  public DefaultDrive(SwerveDrive subsystem1, Vision subsystem2, Joystick driver, double multiplier) {
    requires(subsystem1);
    requires(subsystem2);
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = subsystem1;
    m_vision = subsystem2;
    m_joystick = driver;
    m_xSpeed = 0;
    m_ySpeed = 0;
    m_rot = 0;
    m_fieldRelative = true;
    this.multiplier = multiplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    heading = m_drive.getAngle().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // deadzone
    SmartDashboard.putNumber("MODE", mode);
    if (m_drive.getGyroReset()) {
      heading = m_drive.getAngle().getDegrees();
      m_drive.setGyroReset(false);
    }

    m_xSpeed = 0;
    m_ySpeed = 0;
    m_rot = 0;

    if (!(m_joystick.getRawButtonPressed(1))) {
    // y should be 0 when robot is facing ^ (and intake is facing driver station)
    // x should be negative when intake facing driver station %
    if (Math.abs(m_joystick.getRawAxis(0)) > 0.1) {
      m_ySpeed = m_joystick.getRawAxis(0) * 0.5 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    }
    if (Math.abs(m_joystick.getRawAxis(1)) > 0.1) {
      m_xSpeed = m_joystick.getRawAxis(1) * 0.5 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
    }
    if (Math.abs(m_joystick.getRawAxis(4)) > 0.2) {
      m_rot = m_joystick.getRawAxis(4) * 0.5 * (Math.PI);
    }
    } else {
      if (Math.abs(m_joystick.getRawAxis(0)) > 0.05) {
        m_ySpeed = m_joystick.getRawAxis(0) * 0.05 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
      }
      if (Math.abs(m_joystick.getRawAxis(1)) > 0.05) {
        m_xSpeed = m_joystick.getRawAxis(1) * 0.05 * Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond;
      }
      if (Math.abs(m_joystick.getRawAxis(4)) > 0.05) {
        m_rot = m_joystick.getRawAxis(4) * 0.05 * (Math.PI);
      }
    }

    double curHead = m_drive.getAngle().getDegrees();


    if (m_rot == 0) {
      m_drive.drive(m_xSpeed, m_ySpeed, pid.calculate(curHead, heading), m_fieldRelative);
      
    } else {
      m_drive.drive(m_xSpeed, m_ySpeed, m_rot, m_fieldRelative);
      heading = m_drive.getAngle().getDegrees();
    }

    // fill with correct button
    if (m_joystick.getRawButtonPressed(6)) {
      // to zero all wheels
      SmartDashboard.putNumber("BUTTON PRESSED", 5);
    }
  }
    // A - for vision
    else {
      m_vision.updateTargets();
      double rot = visionController.calculate(m_vision.getYaw(), 0);
      m_drive.drive(0, 0, m_rot, m_fieldRelative);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
