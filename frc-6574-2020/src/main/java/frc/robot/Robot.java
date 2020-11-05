/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
<<<<<<< Updated upstream

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
=======
import edu.wpi.first.wpiutil.net.PortForwarder;
import edu.wpi.cscore.UsbCamera;
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private UsbCamera camera;
  private CameraServer cameraServer;
  //320 x240
  // 20 compression
 // private String gameData;
>>>>>>> Stashed changes

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
<<<<<<< Updated upstream
 //   RobotContainer.compressor.start(); //compressor init code
=======
    RobotContainer.compressor.start();   //compressor init code
    cameraServer = CameraServer.getInstance();
    camera = cameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);
    camera.setFPS(15);
 //   camera.
 //   camera.startAutomaticCapture();

//    PortForwarder.add(5800,"limelight.local",5800);
//    PortForwarder.add(5801,"limelight.local",5801);
//    PortForwarder.add(5805,"limelight.local",5805);

>>>>>>> Stashed changes
    // documentation says this is "true" by default, so commenting out
    // RobotContainer.compressor.setClosedLoopControl(true);
  

      }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(robotContainer.getAutonomousCommand());
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
//    CommandScheduler.getInstance().cancelAll();
    RobotContainer.driveTrain.stop();
    if (!CommandScheduler.getInstance().isScheduled(robotContainer.arcadeDrive)) {
      CommandScheduler.getInstance().schedule(robotContainer.arcadeDrive);

  
    //  RobotContainer.limelight.ledOff();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

}