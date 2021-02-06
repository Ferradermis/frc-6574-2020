/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.commands.HelperMethods;
import edu.wpi.first.wpilibj.Timer;


public class AutoPlanA2Shoots6 extends InstantCommand {
  
  DriveTrain driveTrain;
  
  // PlanA constants: Plan A starts in front of target, shoots 3 balls, retrieves first 3 balls in trench
  // need to decide if we want to pick up last two balls in trench
  // drive to target, shoot 3-5 balls
  final double PlanAHeading1 = -35.0;
  final double PlanAHeading2 = -23.0;
  final double PlanASideA = 8.0;  //10.0
  final double PlanASideB = 6.0;  //7.0
  final double PlanASideC = 15; //12.5


  public AutoPlanA2Shoots6(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
//    (new TurnTurretAtStart(RobotContainer.turret)).schedule();
driveTrain.driveAlongAngle(1.5, 0.0);  

//AimTurret aimTurret = new AimTurret(RobotContainer.turret);
RobotContainer.shooter.raiseHoodForShooting();
Timer.delay(.2);
RobotContainer.shooter.extendHoodForLongDistance();
RobotContainer.shooter.setVelocity(10500);

// aimTurret.schedule();
//CommandScheduler.getInstance().schedule(aimTurret);


Timer.delay(2.5);   
RobotContainer.shooter.feedAndFire();
Timer.delay(1);
//if (aimTurret.isScheduled()) {
//aimTurret.cancel();
//}
//RobotContainer.shooter.retractHoodforShortDistance();

//RobotContainer.shooter.stopShooter();
RobotContainer.shooter.stopFeeder();
RobotContainer.hopper.turnOff();
//RobotContainer.turret.resetTurretForward();
//RobotContainer.shooter.lowerHoodForTrench();

    // Shoot
    //    RobotContainer.shoot.schedule();  

    driveTrain.turnToHeading(-60);
    driveTrain.driveAlongAngle(6, -60); 
    driveTrain.turnToHeading(0.0);  
    RobotContainer.intake.deploy();
    // use new DriveAlongAngle command with slower speed
    driveTrain.driveAlongAngle(9, 0.0); 
    driveTrain.driveAlongAngle(3, 0.0); 
    driveTrain.driveAlongAngle(3, 0.0); 
    // driveTrain.driveAlongAngle(2.0, 0.0); 
    driveTrain.turnToHeading(-28);
    RobotContainer.intake.retract();
    driveTrain.driveAlongAngle(-12.75, -28); 
    RobotContainer.intake.deploy();
    driveTrain.turnToHeading(0.0);
//    RobotContainer.shoot.schedule();    
   
//AimTurret aimTurret = new AimTurret(RobotContainer.turret);
// aimTurret.schedule();
//CommandScheduler.getInstance().schedule(aimTurret);


//Timer.delay(3);   
RobotContainer.shooter.feedAndFire();
Timer.delay(2);
//if (aimTurret.isScheduled()) {
//aimTurret.cancel();
//}
RobotContainer.shooter.retractHoodforShortDistance();

RobotContainer.shooter.stopShooter();
RobotContainer.shooter.stopFeeder();
RobotContainer.hopper.turnOff();
//RobotContainer.turret.resetTurretForward();
RobotContainer.shooter.lowerHoodForTrench();

    HelperMethods.allAutoEnd();
  }
}