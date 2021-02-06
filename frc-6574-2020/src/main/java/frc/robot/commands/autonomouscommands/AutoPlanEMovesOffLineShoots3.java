/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.HelperMethods;
import frc.robot.commands.turretcommands.AimTurret;
import frc.robot.subsystems.DriveTrain;


public class AutoPlanEMovesOffLineShoots3 extends InstantCommand {
  
  DriveTrain driveTrain;

  public AutoPlanEMovesOffLineShoots3(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HelperMethods.allAutoStart();
 
    //   TurnTurretAtStart turnTurretAtStart = new TurnTurretAtStart(RobotContainer.turret);
    driveTrain.driveAlongAngle(1.5,0);
    // Shoot
   // RobotContainer.shoot.schedule(); 
//  THIS IS THE CODE THAT "worked" AT DULUTH

     AimTurret aimTurret = new AimTurret(RobotContainer.turret);
     RobotContainer.shooter.raiseHoodForShooting();
     Timer.delay(.2);
     RobotContainer.shooter.extendHoodForLongDistance();
     Timer.delay(.2);
     RobotContainer.shooter.setVelocity(10500);

     Timer.delay(1);
    // aimTurret.schedule();
     CommandScheduler.getInstance().schedule(aimTurret);


     Timer.delay(3);   
     RobotContainer.shooter.feedAndFire();
     Timer.delay(2);
     if (aimTurret.isScheduled()) {
   aimTurret.cancel();
     }
     RobotContainer.shooter.retractHoodforShortDistance();

     RobotContainer.shooter.stopShooter();
     RobotContainer.shooter.stopFeeder();
     RobotContainer.hopper.turnOff();
    //RobotContainer.turret.resetTurretForward();
     RobotContainer.shooter.lowerHoodForTrench();
// BUT THIS IS THE CODE THAT SHOULD WORK
  //  Shoot myShoot = new Shoot(RobotContainer.shooter);
  //  myShoot.schedule();      
 //   CommandScheduler.getInstance().schedule(RobotContainer.shoot);
 //           Timer.delay(8); // or whatever number is needed to make sure shots taken
 //     if (RobotContainer.shoot.isScheduled()){
 //       RobotContainer.shoot.cancel();
 //     }

    HelperMethods.allAutoEnd();
  }
}