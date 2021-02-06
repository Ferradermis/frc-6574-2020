/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomouscommands.AutoPlanA2Shoots6;
import frc.robot.commands.autonomouscommands.AutoPlanAShoots6;
import frc.robot.commands.autonomouscommands.AutoPlanBShoots5;
import frc.robot.commands.autonomouscommands.AutoPlanCMovesOffLine;
import frc.robot.commands.autonomouscommands.AutoPlanDShoots8;
import frc.robot.commands.autonomouscommands.AutoPlanEMovesOffLineShoots3;
import frc.robot.commands.climbercommands.ClimbUpandDown;
import frc.robot.commands.drivetraincommands.ArcadeDrive;
import frc.robot.commands.shootercommands.ShootCommand;
import frc.robot.commands.shootercommands.StopShooting;
import frc.robot.commands.turretcommands.AimTurret;
import frc.robot.commands.turretcommands.TurnTurret;
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
  /** */ 
  public static final OI oi = new OI(); //Phase out
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Hopper hopper = new Hopper();
  public static final Turret turret = new Turret();
  public static final Climber climber = new Climber();

  public static final Compressor compressor = new Compressor();
  
  //public static Spark leds = new Spark(0);
  
  //Commands
  public final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain);
  public final TurnTurret turnTurret = new TurnTurret(turret);
  public static final AimTurret aimTurret = new AimTurret(turret);
  public static final ClimbUpandDown climb = new ClimbUpandDown(climber);

  public static SendableChooser<CommandBase> autochooser = new SendableChooser<CommandBase>();
  public static SendableChooser<String> allianceChooser = new SendableChooser<String>();

  public RobotContainer() {

    driveTrain.setDefaultCommand(arcadeDrive);
    turret.setDefaultCommand(turnTurret);
  //  climber.setDefaultCommand(climb);

    SmartDashboard.putNumber("Delay Start of Auto: ", 0.0);
    autochooser.addOption("(C)Move off Initiation line", new AutoPlanCMovesOffLine(driveTrain));
    autochooser.addOption("(A) Target S3 + I3 Trench + S3 balls", new AutoPlanAShoots6(driveTrain));
    autochooser.addOption("(A2) Target S3, back up, + I3 Trench + S3 balls", new AutoPlanA2Shoots6(driveTrain));
    autochooser.addOption("(B)Opponent trench I2 S5 balls", new AutoPlanBShoots5(driveTrain));
    autochooser.addOption("(D)Front of Trench S3 + I5 + S5", new AutoPlanDShoots8(driveTrain));
    autochooser.addOption("(E)Move off I Line S3", new AutoPlanEMovesOffLineShoots3(driveTrain));
    SmartDashboard.putData("Autonomous Chooser", autochooser);

    allianceChooser.setDefaultOption("Red Alliance (pipeline)", "red");    
    allianceChooser.addOption("Blue Alliance (pipeline)", "blue");
    SmartDashboard.putData("Alliance (pipeline)", allianceChooser);    

//    SmartDashboard.putNumber("User entered Shooter % Speed", 0.5);
    SmartDashboard.putNumber("User entered Shooter Velocity", 10000);
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  //-----Driver Controls-----\\
  oi.driver_rightBumper.whenPressed(()->intake.deployOrRetract());
  oi.driver_leftTrigger.whenPressed(()->intake.reverseOn()).whenReleased(()->intake.turnOn()); 
  oi.driver_yButton.whenPressed(()->turret.limelight.ledOn());
  oi.driver_xButton.whenPressed(()->turret.limelight.ledOff());
  oi.driver_upDpad.whenPressed(()->climber.moveElevatorStaticUp());
  oi.driver_upDpad.whenReleased(()->climber.stopElevator());
  oi.driver_downDpad.whenPressed(()->climber.moveElevatorStaticDown());
  oi.driver_downDpad.whenReleased(()->climber.stopElevator());

  //-----Operator Controls-----\\    
  oi.operator_aButton.toggleWhenPressed(climb, true);  // schedules ClimbUpAndDown for endgame
  oi.operator_rightTrigger.whenPressed(new ShootCommand(shooter)).whenReleased(new StopShooting(shooter));
  
  oi.operator_rightBumper.whenPressed(()->hopper.turnOnForIntake())
                          .whenReleased(()->hopper.turnOff());

  oi.operator_leftBumper.whenPressed(()->hopper.reverseForIntake())
                        .whenReleased(()->hopper.turnOff());

  oi.operator_upDpad.whenPressed(()->shooter.raiseHoodForShooting());
  oi.operator_downDpad.whenPressed(()->shooter.lowerHoodForTrench());
  oi.operator_rightDpad.whenPressed(()->shooter.extendHoodForLongDistance());
  oi.operator_leftDpad.whenPressed(()->shooter.retractHoodforShortDistance());
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
