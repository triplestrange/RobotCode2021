
package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;

// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import frc.robot.vision.GripPipeline;

// import org.opencv.core.Rect;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.vision.VisionRunner;
// import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Double[] coords;
  // private CANCoder hoodEncoder = new CANCoder(0);
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  private FileWriter fw;
  private File f;
  private BufferedWriter bw;

  // CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.getInstance().startAutomaticCapture().setResolution(320, 160);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    xEntry = table.getEntry("X");
    yEntry = table.getEntry("Y");
    

  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    // SmartDashboard.putNumber("hoodEncoder", hoodEncoder.getPosition());
    // SmartDashboard.putNumber("hoodVoltage", hoodEncoder.getBusVoltage());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    RobotContainer.swerveDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); //0.053, .8539
    RobotContainer.theta.reset(0);
    
    
    try {
      f = new File("/home/lvuser/Output.txt");

      if (!f.exists()) {
        f.createNewFile();
      }
      fw = new FileWriter(f);
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    bw = new BufferedWriter(fw);

    try {
      bw.write("AUTOS");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // String  trajectoryJSON = "output/Straight123.wpilib.json";
    //     Trajectory trajectory = new Trajectory();
    //     try {
    //         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //     } catch (IOException ex) {
    //         DriverStation.reportError("Unable to open trajectory" + trajectoryJSON, ex.getStackTrace());
    //     }


    m_autonomousCommand = m_robotContainer.getAutonomousCommand(null);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      Scheduler.getInstance().add(m_autonomousCommand);
    }

  }

  @Override
  public void autonomousPeriodic() {    
    Double[] coords = {RobotContainer.swerveDrive.getPose().getX(),
      RobotContainer.swerveDrive.getPose().getY()};
    SmartDashboard.putNumberArray("AUTO COORDS", coords);
    
    try {
      f = new File("/home/lvuser/Output.txt");

      if (!f.exists()) {
        f.createNewFile();
      }
      fw = new FileWriter(f);
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    bw = new BufferedWriter(fw);

    try {
      bw.write("AUTOS");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // @Override
  // public void autonomousPeriodic() {
  //   try {
  //     bw.write(RobotContainer.getCoords());
  //     bw.close();
  //   } catch (IOException e) {
  //     e.printStackTrace();
  //   }
  // }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // RobotContainer.swerveDrive.zeroHeading();
  }

  double x = 0;
  double y = 0;
  @Override
  public void teleopPeriodic() {
    xEntry.setDouble(x);
    yEntry.setDouble(y);

    x += 0.05;
    y += 1.0;
  }

  @Override
  public void testInit() {
    Scheduler.getInstance().removeAll();
  }

  @Override
  public void testPeriodic() {
  }
}
