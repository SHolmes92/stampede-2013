/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 *
 * @author Robotics
 */
public class Winch {

    Talon w1;
    Talon w2;
    Encoder rightWinchEncoder;
    Encoder leftWinchEncoder;
    Joystick gamepad;

    public void init() {
        rightWinchEncoder = new Encoder(1, 8, 1, 9);
        leftWinchEncoder = new Encoder(1, 10, 1, 11);
        w1 = new Talon(5);
        w2 = new Talon(7);
        rightWinchEncoder.start();
        leftWinchEncoder.start();


    }

    public void handler() {
    }

    public void ui() {
        if (gamepad.getRawButton(1)) {
            w1.set(gamepad.getY() * 0.6);
        } else {
            w1.stopMotor();
        }
        if (gamepad.getRawButton(3)) {
            w2.set(gamepad.getY() * 0.6);
        } else {
            w2.stopMotor();
        }
    }
}
