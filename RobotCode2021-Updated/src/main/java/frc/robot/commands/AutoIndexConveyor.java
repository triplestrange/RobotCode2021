/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class AutoIndexConveyor extends InstantCommand {
  /**
   * Creates a new ConveyorAutoIndex.
   */
  public AutoIndexConveyor(Conveyor subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(subsystem::autoIndex, subsystem);
  }

}
