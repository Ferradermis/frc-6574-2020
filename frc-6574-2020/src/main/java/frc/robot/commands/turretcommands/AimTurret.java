/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class AimTurret extends CommandBase {

  public static double turnKP = .03;
  public static double simpleFF =.04;
  public static double threshold = .75;
  public static double offset = 0;//-.35;
  // private double MAXROTATION = 45;

  /**
   * Creates a new AimTurret command.
   */
  public AimTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (RobotContainer.turret.limelight.hasTarget()) {
      double angleX = RobotContainer.turret.limelight.getAngleX();

      if (angleX > threshold) {
        RobotContainer.turret.turn((angleX * turnKP) + simpleFF); // copysign Deleted (what is this), .04 added as simple feedforward

      }
      else if (angleX < -threshold) {
        RobotContainer.turret.turn((angleX * turnKP) - simpleFF);
      } else {
        RobotContainer.turret.stopTurning();
      }
    } else {
      RobotContainer.turret.stopTurning();
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
    return Limelight.aimedAtTarget();
  }
}
