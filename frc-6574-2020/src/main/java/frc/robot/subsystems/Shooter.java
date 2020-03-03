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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

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

  public double hoodNeededDistance = 360.0; // distance at which we raise hood

  public Shooter() {
    configureMotors();
    lowerHoodForTrench();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    SmartDashboard.putNumber("Actual Shooter Velocity: ", shooterLeft.getSelectedSensorVelocity());
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
    // so 10 ft would set velocity to 8533 encoder units per 100 ms
    double targetVelocity_UnitsPer100ms = distance * 250.0 * 2048 / 600;
		shooterLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public boolean shooterReady(double distance) {
    double tolerance = 50;
    double targetVelocity_UnitsPer100ms = distance * 250.0 * 2048 / 600;
    
    return (shooterLeft.getSelectedSensorVelocity() >= targetVelocity_UnitsPer100ms - tolerance);
  }

  public void feedAndFire() {
    feeder.set(1);
    RobotContainer.hopper.turnOnForShooting();
  }
  public void stopFeeder() {
    feeder.set(0);  
  }

  public void stopFiring()
  {
    stopFeeder();
    RobotContainer.hopper.turnOff();
  }

  public void stopShooter() {
    shooterLeft.set(ControlMode.Velocity, 0);
  }
 
  public void raiseHoodForShooting() {
      hoodTrench.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerHoodForTrench() {
    hoodAngle.set(DoubleSolenoid.Value.kReverse);
    hoodTrench.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendHoodForLongDistance() {
    // only extend distance hood if trenchHood raised and hoodAngle is down
    if ((hoodTrench.get() == DoubleSolenoid.Value.kForward) && 
              (hoodAngle.get() == DoubleSolenoid.Value.kReverse)) {
      hoodAngle.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void retractHoodforShortDistance() {
    // only retractHood if already extended
    if (hoodAngle.get() == DoubleSolenoid.Value.kForward) {
      hoodAngle.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void testspin(){
//    shooterLeft.set(ControlMode.PercentOutput, SmartDashboard.getNumber("User Entered Shooter % Speed", .5));
    shooterLeft.set(ControlMode.Velocity, SmartDashboard.getNumber("User entered Shooter Velocity", 5000));
//   shooterLeft.set(ControlMode.Velocity, 5000);

}
  
  public void teststop(){
    shooterLeft.set(ControlMode.PercentOutput,0);  
  }

  private void configureMotors() {
    // Set up motors
    double rampRate = 0.2; //time in seconds to go from 0 to full throttle; 0.2 is selected on feel by drivers for 2019
    //int currentLimit = 35; 
    int feederCurrentLimit = 35; 

    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterRight.setInverted(true);
    shooterRight.follow(shooterLeft);

    shooterLeft.configOpenloopRamp(rampRate);
    shooterRight.configOpenloopRamp(rampRate);
    shooterLeft.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE
    shooterRight.setNeutralMode(NeutralMode.Coast); // MAKE SURE WE ARE IN COAST MODE

    // SEE CODE FROM: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java

    /* Config sensor used for Primary PID [Velocity] */
    shooterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);

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
// PEAK SENSOR VELOCITY on 2020 ROBOT is:
// (kMaxRPM  / 600) * (kSensorUnitsPerRotation)
// PEAK RPM of Motor = 6380 RPM
// PEAK SENSOR VELOCITY = 6380 / 600 * 2048  = 21,777
// CURRENT Gear Ratio = 1.25 : 1
// PEAK RPM of Wheel = 7975 RPM

    double kF = 1023 / 21777; // This equals: 0.047
    double kP = 0.1;
    double kI = 0.001;
    double kD = 5;
    shooterLeft.config_kF(0, kF, 20);
    shooterLeft.config_kP(0, kP, 20);
    shooterLeft.config_kI(0, kI, 20);
    shooterLeft.config_kD(0, kD, 20);

    feeder.setOpenLoopRampRate(rampRate);
    feeder.setSmartCurrentLimit(feederCurrentLimit);

    // no current limit on the shooter right now

  }
}