// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class VisionCommand extends Command {
  private final SwerveDrive m_drive;
  private final Vision m_vision;
  private PIDController controller = new PIDController(.1, 0, 0);
  private double rotationSpeed;

  public VisionCommand(SwerveDrive subsystem1, Vision subsystem2) {
    requires(subsystem1);
    m_drive = subsystem1;
    m_vision = subsystem2;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_vision.updateTargets();
    rotationSpeed = controller.calculate(m_vision.getYaw(), 0);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
