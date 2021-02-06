/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetraincommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {

  private DriveTrain driveTrain;
  final double THROTTLE = 1; // controls speed via joystick; useful for test driving
                              // set to 1 for normal drive speed

  public ArcadeDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  // NOTE  ALL Limelight code and buttonpressed tests should move to own commands

  @Override
  public void execute() {
    double steer_cmd=0;
    double drive_cmd=0;

    // Read data from joystick and drive per joystick positioning
    double y = RobotContainer.oi.getDriverLeftY();
    double x = -RobotContainer.oi.getDriverRightX(); 
  
    drive_cmd = Math.pow(y, 1);       // cubing y makes it more "sensitive", maybe need to adjust deadbands
    steer_cmd = Math.pow(x, 1) / 2; // cubing x and /2 makes it more "sensitive", maybe need to adjust deadbands
  
    // throttle is constant that controls "speed" of robot; helpful in testing in small areas
    driveTrain.arcadeDrive(drive_cmd*THROTTLE, steer_cmd*THROTTLE);   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}