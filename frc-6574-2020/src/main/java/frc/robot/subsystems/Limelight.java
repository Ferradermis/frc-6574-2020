/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Limelight camera aboard the robot.
 */
public class Limelight extends SubsystemBase {

    public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight()
    {
        ledOn();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Limelight has target?", hasTarget());
      SmartDashboard.putNumber("Limelight X", getAngleX());
      SmartDashboard.putNumber("Limelight Y", getAngleY());
      SmartDashboard.putNumber("Limelight Y", getSkew());
      SmartDashboard.putNumber("Distance to target: ", getDistanceToTarget(-1));
      SmartDashboard.putNumber("Limelight Pipeline:", limelight.getEntry("pipeline").getDouble(3));

    }
  

    public void setTarget(int pipeline){
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Returns whether or not the Limelight has a valid target
     * in view, as identified by its targeting parameters.
     * 
     * @return  whether or not there is target in view
     */
    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Turns on the Limelight's LEDs.
     */
    public void ledOn() {
        limelight.getEntry("ledMode").setNumber(3);
    }

    /**
     * Turns off the Limelight's LEDs.
     */
    public void ledOff() {
        limelight.getEntry("ledMode").setNumber(1);
    }

    /**
     * Returns the difference between the center of the camera's
     * view and the target's center.
     * 
     * @return  a double containing the difference in degrees (in the range of -29.8 to 29.8)
     */
    public double getAngleX() {
        return limelight.getEntry("tx").getDouble(0);
    }
   /**
     * Returns the difference between the center of the camera's
     * view and the target's center.
     * 
     * @return  a double containing the difference in degrees (in the range of -24.85 to 24.85)
     */
    public double getAngleY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getSkew() {
        return limelight.getEntry("ts").getDouble(0);
    }


    public double getDistanceToTarget(double defaultDistance) {
        if (!hasTarget()) {
            return defaultDistance;
        }

        //All calculations are in centimeters
        final double h2 = 86.36; //height of target
        final double h1 = 25.4; //height of camera
        // NOTE in final code, just calculate h2 - h1 and set a variable    
        final double A1 = 23.5; //Angle of camera relative to ground
    
        double angleY = getAngleY();
        
        // calculate currentDistance from target
        return (h2-h1)/Math.tan((angleY+A1)*Math.PI/180);
      }
    
      public boolean aimedAtTarget() {       
        final double tolerance = 1;
        return (Math.abs(getAngleX()) <= tolerance);
      }
    
}