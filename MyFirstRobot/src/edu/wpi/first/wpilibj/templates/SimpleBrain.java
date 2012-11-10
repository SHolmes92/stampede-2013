/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class SimpleBrain extends SimpleRobot {
    RobotDrive drive = new RobotDrive(1,2,3,4);
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
           drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
           drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
           drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
           drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    
            while (true && isOperatorControl() && isEnabled()) // loop until change 
        {
            drive.mecanumDrive_Cartesian(rightStick.getX(), rightStick.getY(), leftStick.getX(), (double) 0.0 );
        }
    }
}

