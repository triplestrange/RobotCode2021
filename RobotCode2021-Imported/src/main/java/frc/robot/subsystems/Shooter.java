/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
  private final CANSparkMax shooter1, shooter2;
  private final Servo hoodServo;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint, speed, hoodPos;
  protected static final double kDefaultMaxServoPWM = 1;
  protected static final double kDefaultMinServoPWM = 0;

  public Shooter() {

    shooter1 = new CANSparkMax(12, MotorType.kBrushless);
    shooter2 = new CANSparkMax(13, MotorType.kBrushless);
    hoodServo = new Servo(0);
    
    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(60);
    shooter1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(60);
    shooter2.follow(shooter1, true);
    shooter2.burnFlash();

    m_encoder = shooter1.getEncoder();
    m_pidController = shooter1.getPIDController();
    kP = 30;
    kI = 0;
    kD = 0; 
    // kDf = 0.5;
    kIz = 0; 
    kFF = 1.0/5676.0; 
    kMaxOutput = 1; 
    kMinOutput = 0.8;
    maxRPM = 5676.0;
    speed = 4550.0;
    // hoodPos = .25;

    // TODO: set limits

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter Velocity", speed);
    // SmartDashboard.putNumber("Hood Position", hoodPos);
  }

  public void runShooter() {
    setPoint = SmartDashboard.getNumber("Shooter Velocity", 1000);

    m_pidController.setP(30);
    m_pidController.setReference(setPoint, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("OutputCurrent", shooter1.get());
    
  }

  public void stopShooter() {
    shooter1.set(0);
  }

  


  public void runHood(double pos) {
    double currentPos = hoodServo.get();
    double hoodDir = 0;
    if (pos == -1) {
      hoodServo.set(currentPos - 0.01);
    } else if (pos == 1) {
      hoodServo.set(currentPos + 0.01);
    } else {
      hoodServo.set(pos);
    }
  }

  public void periodic() {
    
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("SHOOTER HOOD POS", hoodServo.get());
    SmartDashboard.putNumber("SHOOTER HOOD ANGLE", hoodServo.getAngle());
  }

  public boolean atSpeed() {
    return Math.abs(setPoint - m_encoder.getVelocity()) < 300; // play with the number (go up to 1,000)
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub

  }
}








