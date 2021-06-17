// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.io.*;
import java.util.*;
import java.util.HashMap.*;
import org.json.simple.JSONObject;

/** Add your docs here. */
public class Remote extends Subsystem {
  public static void main(String args) throws FileNotFoundException {
    JSONObject obj = new JSONObject();

    obj.put("name", "foo");
    obj.put("num", new Integer(100));
    obj.put("balance", new Double(1000.21));
    obj.put("is_vip", new Boolean(true));

    System.out.print(obj);
  }
  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
