/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Robotics
 */
public class Shooter {

    DigitalInput shooterWheelSensor;
    Counter shooterCounter;
    double targetRPM;
    double predictedPower;
    double actualPower;
    boolean lastIncreaseButton;
    boolean lastDecreaseButton;
    double shooterRPM;
    double RPMError;
    Timer shooterTimer;
    Joystick joystick;
    Talon shooterTalon;
    int increaseRPMButton,
            decreaseRPMButton,
            offButton, 
            shootingRPMButton, 
            towerShootingRPMButton; 

    public void init() {
        shooterWheelSensor = new DigitalInput(5);
        shooterCounter = new Counter(shooterWheelSensor);
        shooterCounter.start();
        targetRPM = 0;
        actualPower = 0; 
        shooterTimer = new Timer();
        shooterTimer.start();
        shooterTalon = new Talon(6);
    }

    public void handler() {

        predictedPower = targetRPM * (1.0 / 1500.0);
        RPMError = targetRPM - shooterRPM;
        double powerCorrection;
        powerCorrection = (RPMError * (0.4 / 1400.0)) * 1;
        actualPower = predictedPower + powerCorrection;

        if (shooterTimer.get() > 0.75) {
            shooterRPM = (shooterCounter.get() / 4.0 * 60.0 / 0.75);
            shooterCounter.reset();
            shooterTimer.reset();
        }
        //shooterTalon.set(actualPower); 
        shooterTalon.set(predictedPower); 
    }

    public void ui() {
        boolean button;

        // handle speed increase button
        button = joystick.getRawButton(increaseRPMButton);
        if (button) {
            // push detection
            if (lastIncreaseButton == false) {
                targetRPM = targetRPM + 100;
                if (targetRPM > 1500) {
                    targetRPM = 1500;
                }
            }
        }
        lastIncreaseButton = button;

        // handle speed decrease button
        button = joystick.getRawButton(decreaseRPMButton);
        if (button) {
            // push detection
            if (lastDecreaseButton == false) {
                targetRPM = targetRPM - 100;
                if (targetRPM < 0) {
                    targetRPM = 0;
                }
            }
        }
        lastDecreaseButton = button;
        
        if(joystick.getRawButton(offButton)){
            targetRPM = 0; 
        }
        if(joystick.getRawButton(shootingRPMButton)){
            targetRPM = 1800; 
        }
        if(joystick.getRawButton(towerShootingRPMButton)){
            targetRPM = 1800; 
        }

    }

    public void setTargetRPM(double tRPM) {
        targetRPM = tRPM;
    }
}
