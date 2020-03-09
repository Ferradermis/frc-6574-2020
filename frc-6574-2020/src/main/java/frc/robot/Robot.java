/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private UsbCamera camera;
  private CameraServer cameraServer;
  MjpegServer mjpegServer;
  //320 x240
  // 20 compression
 // private String gameData;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    RobotContainer.compressor.start();   //compressor init code
    
    cameraServer = CameraServer.getInstance();
    camera = new UsbCamera("USB Camera 0", 0);
    cameraServer.addCamera(camera);
    mjpegServer = cameraServer.addServer("mjpeg USB Camera");
    mjpegServer.setSource(camera);
    mjpegServer.setCompression(30);
    mjpegServer.setResolution(320, 240);
    mjpegServer.setFPS(15);
  
// THIS IS WHAT WORKED IN DULUTH
//  camera = cameraServer.startAutomaticCapture();
//  camera.setResolution(320,240);
//  camera.setFPS(15);
//  NOT SURE IT WAS ACTUALLY SETTING RESOLUTION AND FPS
//    PortForwarder.add(5800,"limelight.local",5800);
//    PortForwarder.add(5801,"limelight.local",5801);
//    PortForwarder.add(5805,"limelight.local",5805);

    // documentation says this is "true" by default, so commenting out
    // RobotContainer.compressor.setClosedLoopControl(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    RobotContainer.turret.setTarget(robotContainer.getAlliance());
    CommandScheduler.getInstance().schedule(robotContainer.getAutonomousCommand());
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll(); // make sure auto's are stopped

    // make sure default commands are scheduled
    RobotContainer.driveTrain.stop();
    if (!CommandScheduler.getInstance().isScheduled(robotContainer.arcadeDrive)) {
      CommandScheduler.getInstance().schedule(robotContainer.arcadeDrive);
    }
    if (!CommandScheduler.getInstance().isScheduled(robotContainer.turnTurret)) {
        CommandScheduler.getInstance().schedule(robotContainer.turnTurret); 
        
    RobotContainer.shooter.defaultShooterOn();
    }
  }

  @Override
  public void teleopPeriodic() {
    // listen to DriverStation to get data for Control Panel color
    //    gameData = DriverStation.getInstance().getGameSpecificMessage();

  }

}