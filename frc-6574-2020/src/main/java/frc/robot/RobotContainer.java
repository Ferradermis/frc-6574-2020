/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomouscommands.AutonomousMovingPractice;
import frc.robot.commands.autonomouscommands.MoveOffLine;
import frc.robot.commands.autonomouscommands.StraightLineSixBallAuto;
import frc.robot.commands.autonomouscommands.ThreeBallAuto;
import frc.robot.commands.blinkincommands.Rainbow;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
import frc.robot.commands.shootercommands.ShootCommand;
import frc.robot.commands.shootercommands.StopShooting;
import frc.robot.commands.turretcommands.AimTurret;
import frc.robot.commands.turretcommands.TurnTurret;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
  //Subsystems
  public static final OI oi = new OI(); //Phase out
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Hopper hopper = new Hopper();
  public static final Turret turret = new Turret();
  public static final Climber climber = new Climber();
  
  public static final Compressor compressor = new Compressor();

  public static final Blinkin m_blinkin = new Blinkin(0);
    
  //Commands
  public final ArcadeDrive arcadeDrive = new ArcadeDrive();
  public final TurnTurret turnTurret = new TurnTurret();
  public static final AimTurret aimTurret = new AimTurret();

  public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();
  public static SendableChooser<String> allianceChooser = new SendableChooser<String>();

  public RobotContainer() {

    driveTrain.setDefaultCommand(arcadeDrive);
  //  turret.setDefaultCommand(turnTurret);

    SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
    autochooser.addOption("Move off Initiation line", new MoveOffLine(-1));
    autochooser.addOption("ThreeBallAuto", new ThreeBallAuto());
    autochooser.addOption("StraightLineSix", new StraightLineSixBallAuto());
    autochooser.addOption("AutonomousMovingPractice", new AutonomousMovingPractice());
    autochooser.addOption("Rainbow!!!", new Rainbow());
    SmartDashboard.putData("Autonomous Chooser", autochooser);
    allianceChooser.setDefaultOption("Red Alliance (pipeline)", "red");    
    allianceChooser.addOption("Blue Alliance (pipeline)", "blue");
    SmartDashboard.putData("Alliance (pipeline)", allianceChooser);    

    configureButtonBindings();
  }

 
  public void setShooterSpeed(int speed) {
    Shooter.shooterSpeed = speed;
  }

  private void configureButtonBindings() {

    //-----Driver Controls-----\\
    oi.driver_rightBumper.whenPressed(()->intake.deployOrRetract());

    oi.driver_leftTrigger.whenPressed(()->intake.reverseOn()).whenReleased(()->intake.turnOnManual()); 
    oi.driver_yButton.whenPressed(()->climber.moveElevatorStaticUp()).whenReleased(()->climber.stopElevator());
    oi.driver_aButton.whenPressed(()->climber.moveElevatorStaticDown()).whenReleased(()->climber.setClimbertoCurrentPosition());
    oi.driver_bButton.whenPressed(()->shooter.feedAndFire()).whenReleased(new StopShooting());
    oi.driver_xButton.whenPressed(()->climber.moveElevatorStaticDown()).whenReleased(()->climber.stopElevator());

    oi.driver_leftDpad.whenPressed(()->setShooterSpeed(18000));
    oi.driver_downDpad.whenPressed(()->setShooterSpeed(15500));
    oi.driver_upDpad.whenPressed(()->setShooterSpeed(21000));
    oi.driver_rightDpad.whenPressed(()->setShooterSpeed(19500));


    //-----Operator Controls-----\\    
    //oi.operator_aButton.toggleWhenPressed(climb, true);  // schedules ClimbUpAndDown for endgame
    oi.operator_rightTrigger.whenPressed(new ShootCommand()).whenReleased(new StopShooting());
    oi.operator_leftTrigger.whenPressed(()->intake.turnOn())
                            .whenReleased(()->intake.turnOff());
    
    oi.operator_rightBumper.whenPressed(()->hopper.turnOnForIntake())
                            .whenReleased(()->hopper.turnOff());

    oi.operator_leftBumper.whenPressed(()->hopper.reverseForIntake())
                          .whenReleased(()->hopper.turnOff());

    oi.operator_upDpad.whenPressed(()->shooter.raiseHoodForShooting());
    oi.operator_downDpad.whenPressed(()->shooter.lowerHoodForTrench());
    oi.operator_rightDpad.whenPressed(()->shooter.extendHoodForLongDistance());
    oi.operator_leftDpad.whenPressed(()->shooter.retractHoodforShortDistance());
    oi.operator_aButton.whenPressed(new AimTurret()).whenReleased(new InstantCommand(turret::stopAiming, turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autochooser.getSelected();
  }

  public String getAlliance() {
    return allianceChooser.getSelected();
  }

}
