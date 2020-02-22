/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
 
  // hood controller for raising and lowering
   public DoubleSolenoid hoodTrench = new DoubleSolenoid(RobotMap.HOOD_TRENCH_ID1, RobotMap.HOOD_TRENCH_ID2);
   public DoubleSolenoid hoodAngle = new DoubleSolenoid(RobotMap.HOOD_ANGLE_ID2, RobotMap.HOOD_ANGLE_ID1);

  private double MAXROTATION = 45;


  private boolean shooting = false;
  public double hoodNeededDistance = 360.0; // distance at which we raise hood

  public Shooter() {
    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  /*  
    */  
  }

  public void spin(double distance) {
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
  {}
    shooterLeft.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Shooter Speed", .5));
  }
  public void teststop(){
    SmartDashboard.putNumber("Shooter Velocity: ", shooterLeft.getSensorCollection().getIntegratedSensorVelocity());
    shooterLeft.set(ControlMode.PercentOutput,0);
    
  }
  public boolean shooterReady(double distance) {
    double targetVelocity_UnitsPer100ms = distance * 250.0 * 2048 / 600;
    double tolerance = 10;
    return (shooterLeft.getSelectedSensorVelocity() >= targetVelocity_UnitsPer100ms - tolerance);
  }

  public void feedAndFire()
  {
    feeder.set(1);
    Timer.delay(2);
    feeder.set(0);
  }

  public void stopFeeder()
  {
    feeder.set(0);
  }

  public void shoot() {
    shooting = true;
  }

  public void stopShooter() {
    shooting = false;
    shooterLeft.set(ControlMode.Velocity, 0);
  }
 
  public void raiseHoodForShooting()
  {
    hoodTrench.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerHoodForTrench()
  {
    hoodAngle.set(DoubleSolenoid.Value.kReverse);
    hoodTrench.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendHoodForLongDistance()
  {
    // only extend distance hood if trenchHood raised
    if (hoodTrench.get() == DoubleSolenoid.Value.kForward) {
      hoodAngle.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retractHoodforShortDistance()
  {
    hoodAngle.set(DoubleSolenoid.Value.kReverse);
  }

  private void configureMotors(){
    // Set up motors
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
    int currentLimit = 35; 
    int feederCurrentLimit = 35; 

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
//    shooterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

    /**
    * Phase sensor accordingly. 
    * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
//    shooterLeft.setSensorPhase(true);

    /* Config the peak and nominal outputs */
//    shooterLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
//    shooterLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
//    shooterLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
//    shooterLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    double kF = 2048.0/6380.0; // why this
    double kP = 0.25;
    double kI = 0.001;
    double kD = 20;
//    shooterLeft.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
//    shooterLeft.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
//    shooterLeft.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
//    shooterLeft.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);

    feeder.setOpenLoopRampRate(rampRate);
    feeder.setSmartCurrentLimit(feederCurrentLimit);

  }
}