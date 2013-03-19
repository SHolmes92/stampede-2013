/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 *
 * @author Robotics deck controls:
 */
public class Deck {

    Joystick joystick;
    int raiseDeckButton,
            lowerDeckButton,
            shootingPositionButton,
            towerPositionButton;
    Talon angulator;
    double lastDeckAngle;
    DigitalInput lowerLimit;
    DigitalInput upperLimit;
    Encoder angleEncoder;
    private boolean moveUp,
            moveDown,
            deckTopRequest,
            deckBottomRequest;
    public double deckAngle,
            deckAngleOffset,
            targetAngle;
    private static final double deckUpAngle = 61;
    private static final double deckDownAngle = 16;
    private static final double shootingPositionAngle = 45;
    private static final double towerPositionAngle = 50;

    public void init() {
        deckAngle = 0;
        deckAngleOffset = 0;
        targetAngle = 0;

        angulator = new Talon(2, 1);
        lowerLimit = new DigitalInput(2, 2);
        upperLimit = new DigitalInput(2, 1);
        angleEncoder = new Encoder(2, 3, 2, 4);
        angleEncoder.start();
        angleEncoder.setDistancePerPulse(-41.0 / 26.0 * 4);

    }

    public void teleopInit(){
        targetAngle = 0; 
    }
    
    public void autonomousInit(){
        targetAngle = 0; 
    }
    
    public void handler() {

        //handle angle reading 
        if (!lowerLimit.get()) {
            deckAngleOffset = deckDownAngle - angleEncoder.getDistance();
        } else if (!upperLimit.get()) {
            deckAngleOffset = deckUpAngle - angleEncoder.getDistance();
        }
        deckAngle = angleEncoder.getDistance() + deckAngleOffset;

        double angulatorPower;

        // firingPosition = 

        // move up?
        if (moveUp || deckTopRequest) {
            if (upperLimit.get()) {
                angulatorPower = 1;
            } else {
                angulatorPower = 0;
                deckTopRequest = false;
            }
        } else if (moveDown || deckBottomRequest) {
            if (lowerLimit.get()) {
                angulatorPower = -1;
            } else {
                angulatorPower = 0;
                deckBottomRequest = false;
            }
        } else if (targetAngle != 0) {
            if (deckAngle > (targetAngle + 2)) {
                // move down
                angulatorPower = -1;
            } else if (deckAngle < (targetAngle - 2)) {
                // move up
                angulatorPower = 1;
            } else {
                // target angle reached - stop following!
                angulatorPower = 0;
                targetAngle = 0;
            }
        } else {
            angulatorPower = 0;
        }
        angulator.set(angulatorPower);
    }

    public void ui() {
        moveUp = joystick.getRawButton(raiseDeckButton);
        moveDown = joystick.getRawButton(lowerDeckButton);
        if (joystick.getRawButton(shootingPositionButton)) {
            targetAngle = shootingPositionAngle;
        }

        if (joystick.getRawButton(towerPositionButton)) {
            targetAngle = towerPositionAngle;
        }

        // move down wins
        if (moveDown) {
            moveUp = false;
        }
        // any manual request overrides top/down requests
        if (moveUp || moveDown) {
            deckTopRequest = false;
            deckBottomRequest = false;
            targetAngle = 0;
        }
    }

    boolean isAtTop() {
        return !upperLimit.get();
    }

    boolean isAtTargetAngle() {
        return (Math.abs(targetAngle - deckAngle) <= 1);
    }

    void moveToTop() {
        deckTopRequest = true;
    }

    void moveToBottom() {
        deckBottomRequest = true;
    }

    void moveToAngle(double a) {
        targetAngle = a;
    }
}
