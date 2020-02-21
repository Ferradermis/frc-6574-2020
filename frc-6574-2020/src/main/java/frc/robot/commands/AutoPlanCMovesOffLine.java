/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class AutoPlanCMovesOffLine extends InstantCommand {
  /**
   * Creates a new RunAutonomousSequence.
   */
  DriveTrain driveTrain;

  public AutoPlanCMovesOffLine(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = Timer.getFPGATimestamp();
    System.out.println("Running Autonomous Plan ");
    System.out.println("Starting Time:" + Timer.getFPGATimestamp());
    driveTrain.stop();
    driveTrain.resetGyro();

    driveTrain.driveAlongAngle(3, 1, 0); //CODE

  //  driveTrain.stop();    // make sure we are stopped at end of autonomous
    double endTime = Timer.getFPGATimestamp();
    System.out.println("Ending Time:" + endTime);
    System.out.println("Run Time of Autonomous: " + (endTime - startTime));
  }
  
}
