/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
 // private UsbCamera spindexerCamera;
  //private CameraServer cameraServer;
  //MjpegServer mjpegServer;
  // 320 x240
  // 20 compression
  
  // private String gameData;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    SmartDashboard.putData(CommandScheduler.getInstance());

    CameraServer.getInstance().startAutomaticCapture();

    //stops the compressor
    //RobotContainer.compressor.start(); //starts the compressor
    
    //cameraServer = CameraServer.getInstance();
    
    //spindexerCamera = new UsbCamera("spindexerCamera", 0);
    //cameraServer.addCamera(spindexerCamera);
    //mjpegServer = cameraServer.addServer("mjpeg USB Camera");
    //mjpegServer.setSource(spindexerCamera);
    //mjpegServer.setCompression(30);
    //spindexerCamera.setResolution(320, 240);
    //spindexerCamera.setFPS(15);
  
    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //camera.setResolution(640, 480);

// THIS IS WHAT WORKED IN DULUTH
    //spindexerCamera = cameraServer.startAutomaticCapture();

//  NOT SURE IT WAS ACTUALLY SETTING RESOLUTION AND FPS
//    PortForwarder.add(5800,"limelight.local",5800);
//    PortForwarder.add(5801,"limelight.local",5801);
//    PortForwarder.add(5805,"limelight.local",5805);
      Blinkin.lightChaseRed();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
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
        
    RobotContainer.shooter.setVelocity(Shooter.shooterSpeed);
    }
  }

  public void testPeriodic(){
    RobotContainer.compressor.setClosedLoopControl(true);
    RobotContainer.compressor.start();
  }

  @Override
  public void teleopPeriodic() {
    
    
    if (OI.driver_startButton.get()) { //disables shooter and compressor for endgame
      Blinkin.blue();
      RobotContainer.shooter.defaultShooterOff();
      RobotContainer.compressor.setClosedLoopControl(false);
      RobotContainer.compressor.stop();
    }

    else if (OI.driver_backButton.get()) { //enables shooter and compressor for standard teleop
      Blinkin.red();
      RobotContainer.shooter.setVelocity(Shooter.shooterSpeed);
      RobotContainer.compressor.setClosedLoopControl(true);
      RobotContainer.compressor.start();
    }
  }

}