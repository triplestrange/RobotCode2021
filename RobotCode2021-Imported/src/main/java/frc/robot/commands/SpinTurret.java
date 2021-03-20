/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SpinTurret extends InstantCommand {
  public SpinTurret(Turret subsystem1, SwerveDrive subsystem2, Vision subsystem3, int mode, double speed) {
    // mode 1: manual
    // mode 2: auto adjust (without vision)
    // mode 3: vision
    super(subsystem1, () -> {subsystem1.spin(subsystem2, subsystem3, mode, speed);});
    requires(subsystem1);
    requires(subsystem2);
    requires(subsystem3);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
