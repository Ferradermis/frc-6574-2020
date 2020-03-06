/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;


public class AutoPlanEMovesOffLineShoots3 extends InstantCommand {
  
  DriveTrain driveTrain;
  
  // PlanA constants: Plan A starts in front of target, shoots 3 balls, retrieves first 3 balls in trench
  // need to decide if we want to pick up last two balls in trench
  // drive to target, shoot 3-5 balls
  final double PlanAHeading1 = -30.25;
  final double PlanAHeading2 = -23.0;
  final double PlanASideA = 9.156;  //10.0
  final double PlanASideB = 7.0;  //7.0
  final double PlanASideC = 12.5; //12.5


  public AutoPlanEMovesOffLineShoots3(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();

    (new TurnTurretAtStart(RobotContainer.turret)).schedule();
    //).andThen(RobotContainer.shoot).schedule();
    driveTrain.driveAlongAngle(1.5,0);
    // Shoot
    RobotContainer.shoot.withTimeout(8).schedule(); 
    Timer.delay(8);   
    
    HelperMethods.allAutoEnd();
  }
}