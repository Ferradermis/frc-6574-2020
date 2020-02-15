/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // shooter/shooter is two falcons -- built in encoder talonFX
  private WPI_TalonFX shooterLeft = new WPI_TalonFX(RobotMap.SHOOTERLEFT_CAN_ID);
  private WPI_TalonFX shooterRight = new WPI_TalonFX(RobotMap.SHOOTERRIGHT_CAN_ID);

  // feeder neo550's
  private CANSparkMax feeder = new CANSparkMax(RobotMap.FEEDER_CAN_ID, MotorType.kBrushless);
 
  // rotator vexPro775
  private TalonSRX turretRotator = new TalonSRX(RobotMap.TURRET_CAN_ID);


  
  // hood controller for raising and lowering
   public Solenoid hoodTrench = new Solenoid(RobotMap.HOOD_TRENCH_ID);
   public Solenoid hoodAngle = new Solenoid(RobotMap.HOOD_ANGLE_ID);

  private double MAXROTATION = 45;

  Limelight limelight = new Limelight();

  private boolean shooting = false;
  private double hoodNeededDistance = 360.0; // distance at which we raise hood

  public Shooter() {
    configureMotors();
    limelight.ledOn();
    limelight.setTarget(0);
    turretRotator.setSelectedSensorPosition(0); // need to think of best way to do this
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((shooting == true)&&(limelight.hasTarget())) {
      raiseHoodForShooting();
      spin(getDistanceToTarget());
      aim();
    
      if (getDistanceToTarget()>hoodNeededDistance) {
        extendHoodForLongDistance();
      } else {
        retractHoodforShortDistance();
      }

      if (aimed() && shooterReady(getDistanceToTarget())) {
          feedAndFire();
        } else { // shooting, but not aimed or not ready
          stopFeeder();
        }

    } else { // not shooting or no target
        stopShooting(); // stops all motors
    }  
  }

  private void aim()
  {
    double kP = .01;

    // If no target in view; stop and exit
    if (limelight.hasTarget()) {
      double angleX = limelight.getAngleX();
      if (Math.abs(turretRotator.getSelectedSensorPosition())<MAXROTATION) {
        turretRotator.set(ControlMode.PercentOutput, angleX*kP);
      }  else {
        stopShooting();
      }
    } else {
      stopShooting();
    }
  }

  private double getDistanceToTarget() {
    //All calculations are in centimeters
    final double h2 = 86.36; //height of target
    final double h1 = 16.51; //height of camera
    // NOTE in final code, just calculate h2 - h1 and set a variable    
    final double A1 = 3.19; //Angle of camera relative to ground

    double angleY = limelight.getAngleY();
    
    // calculate currentDistance from target
    return (h2-h1)/Math.tan((angleY+A1)*Math.PI/180);
  }

  private boolean aimed() {       
    final double tolerance = 0.5;
    return (Math.abs(limelight.getAngleX()) < tolerance);
  }

  private void resetTurretForward() {
    turretRotator.set(ControlMode.Position, 0);
  }

  private void spin(double distance) {
    // need to figure out this formula to set velocity based on distance
    // see the html file linked at the top of this java file
    // distance = parameter passed in; 
    // 250 = "normal" rpm; so if distance = 10, this would set rpms to 2500
    // (note the Falcon 500 has a free speed rpm of 6380RPM/1.5A)
    // 2048 = units per rotation
    // so 500 x 2048 is encoder units per minute
    // 600 = converts those units to units per 100ms
    double targetVelocity_UnitsPer100ms = distance * 250.0 * 2048 / 600;

		shooterLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void testspin(){
    shooterLeft.set(ControlMode.PercentOutput,.25);
  }
  public void teststop(){
    shooterLeft.set(ControlMode.PercentOutput,0);
    
  }
  private boolean shooterReady(double distance) {
    double targetVelocity_UnitsPer100ms = distance * 250.0 * 2048 / 600;
    double tolerance = 10;
    return (shooterLeft.getSelectedSensorVelocity() >= targetVelocity_UnitsPer100ms - tolerance);
  }

  public void feedAndFire()
  {
    feeder.set(.5);
    Timer.delay(.25);
  }

  public void shoot() {
    shooting = true;
  }

  public void stopShooting() {
    shooting = false;
    stopAiming();
    stopShooter();
    stopFeeder();
    retractHoodforShortDistance();
    lowerHoodForTrench();
    resetTurretForward();
    // lower the hood controllers
  }

  private void stopShooter() {
    shooterLeft.set(ControlMode.Velocity, 0);
  }

  public void stopAiming() {
    turretRotator.set(ControlMode.PercentOutput, 0);
  }

  public void stopFeeder()
  {
    feeder.set(0);
  }

  public void raiseHoodForShooting()
  {
    hoodTrench.set(false);
  }

  public void lowerHoodForTrench()
  {
    hoodAngle.set(false);
    hoodTrench.set(true);
  }

    public void extendHoodForLongDistance()
  {
    // only extend distance hood if trenchHood raised
    if (!hoodTrench.get()) {
      hoodAngle.set(true);
    }
  }

  public void retractHoodforShortDistance()
  {
    hoodAngle.set(false);
  }

  public void testTurnTurret()
  {
    System.out.println(turretRotator.getSelectedSensorPosition());
    turretRotator.set(ControlMode.PercentOutput,.1);
    Timer.delay(1);
    turretRotator.set(ControlMode.PercentOutput,0);
    System.out.println(turretRotator.getSelectedSensorPosition());
  }

  private void configureMotors(){
    // Set up motors
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
    int currentLimit = 30; 

    shooterRight.setInverted(true);
    shooterRight.follow(shooterLeft);
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();
    shooterLeft.configOpenloopRamp(rampRate);
    shooterRight.configOpenloopRamp(rampRate);
    shooterLeft.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE
    shooterRight.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE

    // SEE CODE FROM: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java

    /* Config sensor used for Primary PID [Velocity] */
    shooterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

    /**
    * Phase sensor accordingly. 
    * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
    shooterLeft.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    shooterLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    double kF = 2048.0/6380.0; // why this
    double kP = 0.25;
    double kI = 0.001;
    double kD = 20;
    shooterLeft.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
    shooterLeft.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
    shooterLeft.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
    shooterLeft.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);

    feeder.setOpenLoopRampRate(rampRate);
    feeder.setSmartCurrentLimit(currentLimit);
  }
}