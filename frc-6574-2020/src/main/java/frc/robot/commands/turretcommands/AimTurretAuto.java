/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AimTurretAuto extends CommandBase {
  /**
   * Creates a new AimTurret command.
   */
  

  //private double offset = 0;//-.35;
  private double angleX;
  private double errorMagnitude;
  private boolean withinMargin = false;

  // private double MAXROTATION = 45;

  public AimTurretAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
   
    if (RobotContainer.turret.limelight.hasTarget()) {
      angleX = RobotContainer.turret.limelight.getAngleX();
      errorMagnitude = Math.abs(angleX);
      SmartDashboard.putNumber("errorMagnitude", errorMagnitude);
      if (errorMagnitude < AimTurret.threshold) {
        withinMargin = true;
      }
      if (angleX > AimTurret.threshold) {
        RobotContainer.turret.turn((angleX * AimTurret.turnKP) + AimTurret.simpleFF); // copysign Deleted (what is this), .04 added as simple feedforward

      }
      else if (angleX < -AimTurret.threshold) {
        RobotContainer.turret.turn((angleX * AimTurret.turnKP) - AimTurret.simpleFF);
      }
    } 
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopTurning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinMargin;
  }
}
