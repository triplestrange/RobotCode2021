package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {

    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.motor, MotorType.kBrushless);
    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 1);
    private boolean extended = false;
    private double speedIn;
    private double speedOut;

    public Intake() {
        super();
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(11);
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor.burnFlash();
    }

    public void extend(Joystick joystick) {
        intakeSolenoid.set(Value.kForward);
        setExtended(true);
        double speedIn = joystick.getRawAxis(3);
        double speedOut = joystick.getRawAxis(2);

        if (speedIn > 0.1)
            intakeMotor.set(speedIn / 2);
        else if (speedOut > 0.1)
            intakeMotor.set(-speedOut);
        else
            intakeMotor.set(0);
    }


    public void retract() {
        intakeSolenoid.set(Value.kReverse);
        setExtended(false);
    }

    public void runWheels(Joystick joystick) {
        double speedOut = joystick.getRawAxis(2);
        double speedIn = joystick.getRawAxis(3);
        if (getExtended()) {
            if (speedIn > 0.1)
                intakeMotor.set(speedIn / 2);
            else if (speedOut > 0.1)
                intakeMotor.set(-speedOut);
         else
            intakeMotor.set(0);
        }
    }

    //auto intakes
    public void runWheelsAuto() {
        intakeMotor.set(.5);
    }

    public void extendAuto() {
        intakeSolenoid.set(Value.kForward);
        setExtended(true);
        intakeMotor.set(.5);
    }

    public boolean getExtended() {
        return extended;
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

}