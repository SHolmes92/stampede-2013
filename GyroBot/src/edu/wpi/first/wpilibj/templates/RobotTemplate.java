/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    DriverStationLCD driverStation = DriverStationLCD.getInstance();
    
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    
    Gyro gyro = new Gyro(0);
    
    double gyroDrift;   // degrees per second;
    double gyroLastRdg;
    double gyroLastTime;
    
    boolean headingLock;    // heading lock mode
    double heading;         // the locked heading (according to gyro)
    double lastAngle;
    double lastAngleTime;
    
    // sonar
    Ultrasonic sonar;
    boolean sonarLock, sonarLocked;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        driverStation.println(DriverStationLCD.Line.kUser2, 1, "Robot booting...");

        // start the gyro
        gyro.reset();
        // prepare for sample collection for gyro drift correction
        gyroLastRdg = gyro.getAngle();
        gyroLastTime = Timer.getFPGATimestamp();
        gyroDrift = 0;
        
        // initiate the drive system
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    }
    
    // called periodically during disabled
    public void disabledPeriodic() {
        // we should start reading the gyro here to correct for any drift
        // take a drift reading based on the last 5 seconds (MAY ADJUST!!!)
        double dt = Timer.getFPGATimestamp() - gyroLastTime;
        if(dt >= 5){
            // calculate drift
            gyroDrift = (gyro.getAngle() - gyroLastRdg) / dt;
            // save current reading
            gyroLastRdg = gyro.getAngle();
            gyroLastTime = Timer.getFPGATimestamp();
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        
        //--------------------------------------------------------------------
        //  Gyro reading
        //--------------------------------------------------------------------
        
        // calculate drift-adjusted gyro position
        double gyroAngle = gyro.getAngle() + gyroDrift * (Timer.getFPGATimestamp() - gyroLastTime);
        double time = Timer.getFPGATimestamp();
        
        // control - standard or heading-lock

        // get control variables
        double y = 0;   // fwd speed
        double x = 0;   // side motion
        double r = 0;   // rotation

        //--------------------------------------------------------------------
        //  Sonar reading
        //--------------------------------------------------------------------
        double sideSonarDistance = 0;
        if(sonarLock){
            if(sideSonarDistance < 12){
                sonarLocked = true;
            }
            if(sonarLocked){
                if(sideSonarDistance < 9){
                    r = 0.2;
                }
                else if(sideSonarDistance  > 10){
                    r = -0.2;
                }
                else {
                    r = 0;
                }
            }
        }
        
        if(false){
            // lock the heading
            if(!headingLock){
                headingLock = true;
                heading = gyroAngle;
            }
        }
        
        if(!headingLock){
            // standard control
            // joy1 - field drive, joy 2 - rotation speed
        }
        else {
            // heading-lock control
            // joy1 - field drive with fixed heading, joy2 - rotation speed (gyro-controlled)
            // adjust heading based on joy settings
            // scale to maximum degrees per second rate
            heading += 90 * r * (time - lastAngleTime);
            
            // adjust rotation setting to maintain heading
            double hdgError = gyroAngle - heading;
            if(Math.abs(hdgError) > 5){
                // set rotation
                //...
                r = Math.min(1, Math.abs(hdgError / 5)) * ((hdgError >= 0)?1:-1);
            }
        }
        
        lastAngleTime = time;
        
        // now drive
        if(headingLock){
            drive.mecanumDrive_Cartesian(x, y, r, gyroAngle);
        }
        else {
            drive.mecanumDrive_Cartesian(x, y, r, 0.0);
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        // calculate drift-adjusted gyro position
        double gyroAngle = gyro.getAngle() + gyroDrift * (Timer.getFPGATimestamp() - gyroLastTime);
    }
}
