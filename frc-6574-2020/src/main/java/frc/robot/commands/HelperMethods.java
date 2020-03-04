/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class HelperMethods {
    static double autoStartTime;

    public static void allAutoStart() {
      autoStartTime = Timer.getFPGATimestamp();     
      RobotContainer.driveTrain.stop();
      RobotContainer.driveTrain.resetGyro();

      double delay = SmartDashboard.getNumber("Delay", 0.0);
      if (delay > 0.0) {
        Timer.delay(delay);
      }
    }

    public static void allAutoEnd() {
        double endTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("Run Time of Autonomous: ", (endTime - autoStartTime));
    }
}
