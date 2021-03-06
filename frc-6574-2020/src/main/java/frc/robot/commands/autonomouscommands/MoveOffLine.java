/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.HelperMethods;
import frc.robot.RobotContainer;

public class MoveOffLine extends InstantCommand {

  private double distance;

  public MoveOffLine(double distance) {
    this.distance = distance;
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
// (new TurnTurretAtStart(RobotContainer.turret)).schedule();
    RobotContainer.driveTrain.resetPosition();
    RobotContainer.driveTrain.driveAlongAngle(distance, 0);
    
    //driveTrain.arcadeDrive(.25,0);
    //Timer.delay(3);
    //driveTrain.stop();

    HelperMethods.allAutoEnd();
  } 
}