// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Vision extends Subsystem {
  private PhotonCamera _camera;
  private List<PhotonTrackedTarget> _targets;
  private PhotonTrackedTarget _target;
  private boolean _hasTargets;
  private PIDController _controller;
  private PhotonPipelineResult _result;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Vision(PhotonCamera camera) {
     // photonvision.local:5800

     _camera = camera;
     // Set driver mode to on.
    _camera.setDriverMode(true);

    // Change pipeline to 2
    _camera.setPipelineIndex(2);

    updateTargets();

    _controller = new PIDController(.1, 0, 0);
  }

  /* STANDARD FUNCTIONS */
  public void updateTargets() {
    // Get the latest pipeline result.
    _result = _camera.getLatestResult();

    // Check if the latest result has any targets.
    _hasTargets = _result.hasTargets();

    // Get a list of currently tracked targets.
    _targets = _result.getTargets();

    // Get the current best target.
    _target = _result.getBestTarget();

    // Get the pipeline latency.
    // double latencySeconds = getLatencyMillis() / 1000.0; // around 3 ms

    // Blink the LEDs.
    _camera.setLED(LEDMode.kOff);
  }

  public void periodic() {
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Area", getArea());
    SmartDashboard.putNumber("Skew", getSkew());
  }

  public double getYaw() {
    double yaw = _target.getYaw();
    return yaw;
  }

  public double getPitch() {
    double pitch = _target.getPitch();
    return pitch;
  }

  public double getArea() {
    double area = _target.getArea();
    return area;
  }

  public double getSkew() {
    double skew = _target.getSkew();
    return skew;
  }

  public Transform2d getPose() {
    Transform2d pose = _target.getCameraToTarget();
    return pose;
  }
  /* ****************** */

  /* TO BE USED FOR COMMANDS */
  public double getRotationSpeed() {
    updateTargets();
    double rotationSpeed = _controller.calculate(_result.getBestTarget().getYaw(), 0);
    return rotationSpeed;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
