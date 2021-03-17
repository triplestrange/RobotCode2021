// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandGroup {
  /** Creates a new IntakeCommand. */
  private final Intake m_intake;
  
  public IntakeCommand(Intake subsystem) {
    requires(subsystem);
    m_intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runWheelsAuto();
    m_intake.extendAuto();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
