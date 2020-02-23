/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private String gameData;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    RobotContainer.compressor.start(); //compressor init code

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
    CommandScheduler.getInstance().cancelAll(); // make sure auto's are stopped

    // make sure default commands are scheduled
    RobotContainer.driveTrain.stop();
    if (!CommandScheduler.getInstance().isScheduled(robotContainer.arcadeDrive)) {
      CommandScheduler.getInstance().schedule(robotContainer.arcadeDrive);
    }
    if (!CommandScheduler.getInstance().isScheduled(robotContainer.turnTurret)) {
        CommandScheduler.getInstance().schedule(robotContainer.turnTurret);  
    }
  }

  @Override
  public void teleopPeriodic() {
    // listen to DriverStation to get data for Control Panel color
    gameData = DriverStation.getInstance().getGameSpecificMessage();
  }

}