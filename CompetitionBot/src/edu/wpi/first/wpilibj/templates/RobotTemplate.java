/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {

    Deck deck;
    Shooter shooter;
    Launcher launcher;
    DriveTrain driveTrain;
    Winch winch;
    DriverStationLCD driverStation;
    Joystick leftStick;
    Joystick rightStick;
    Joystick gamepad;
    Timer autoTimer;
    int autoState;
    int autoShots;
    double autoBackTime;
    double autoShotAngle;
    double autoShotRPM;
    Preferences autoPrefs;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
      /*  
        autoShots = autoPrefs.getInt("autoShotCount", 3);
        autoBackTime = autoPrefs.getDouble("autoBackTime", 0);
        autoShotAngle = autoPrefs.getDouble("autoShotAngle", 40);
        autoShotRPM = autoPrefs.getDouble("autoShotRPM", 1200);
      */
        // create all objects
        shooter = new Shooter();
        driveTrain = new DriveTrain();
        launcher = new Launcher();
        deck = new Deck();
        winch = new Winch();

        shooter.init();
        driveTrain.init();
        launcher.init();
        deck.init();
        winch.init();

        
        
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        gamepad = new Joystick(3);

        //intialize deck controls 
        deck.joystick = rightStick;
        deck.raiseDeckButton = 3;
        deck.lowerDeckButton = 2;
        deck.shootingPositionButton = 4;
        deck.towerPositionButton = 5;

        //intilize shooter controls 
        shooter.joystick = rightStick;
        shooter.increaseRPMButton = 11;
        shooter.decreaseRPMButton = 10;

        //intialize drive train controls
        driveTrain.rightStick = rightStick;
        driveTrain.leftStick = leftStick;
        driveTrain.approachButton = 4;
        driveTrain.gyroLockButton = 1;

        //intialize winch controls 
        winch.gamepad = gamepad;

        //intialize launcher controls 
        launcher.joystick = rightStick;
        launcher.fireButton = 1;
        launcher.rePushButton = 7;
        launcher.reTapButton = 6;

        autoTimer = new Timer();


    }

    public void disabledInit() {
    }

    // called periodically during disabled
    public void disabledPeriodic() {
    }

    public void teleopInit() {
        driveTrain.teleopInit();
        launcher.teleopInit();
        deck.teleopInit();


    }

    public void teleopPeriodic() {
        driveTrain.handler();
        deck.handler();
        shooter.handler();
        launcher.handler();
        winch.handler();

        displayHandler();

        driveTrain.ui();
        deck.ui();
        shooter.ui();
        launcher.ui();
        // winch.ui(); 

        if (leftStick.getRawButton(2)) {
            //loading configuration 
            deck.moveToBottom();
            shooter.setTargetRPM(0);
        } else if (leftStick.getRawButton(3)) {
            //shooting configuration 
            deck.moveToAngle(45);
            shooter.setTargetRPM(1500);
        }

    }

    public void autonomousInit() {
        driveTrain.drive.setSafetyEnabled(false);

        launcher.autonomousInit();
        autoTimer.start();
        autoState = 0;

        autoShots = 3; 

        deck.autonomousInit();




    }

    public void autonomousPeriodic() {

        driveTrain.handler();
        deck.handler();
        shooter.handler();
        launcher.handler();
        winch.handler();

        //handle sequencing 
        double t = autoTimer.get();

        switch (autoState) {
            case 0: // raise deck, spin up
                deck.moveToTop();
                shooter.setTargetRPM(1200);
                autoState++;
                break;

            case 1: // check if deck up, go to target angle
                if (deck.isAtTop()) {
                    deck.moveToAngle(40);
                    autoState++;
                }
                break;

            case 2: // wait until deck at level
                if (deck.isAtTargetAngle()) {
                    autoState++;
                }
                break;

            case 3: // make sure some min. time has transpired
                if (t >= 3) {
                    autoState++;
                    autoTimer.reset();
                }
                break;

            case 4: // shoot every 2 seconds, 3 times
                if (autoShots > 0) {
                    if (t >= 2) {
                        launcher.fire();
                        autoTimer.reset();
                        autoShots--;
                    }
                } else {
                    autoState++;
                }
                break;

            case 5: // back out
                // drive backwards slowly
                driveTrain.drive(0, -0.4, 0);
                autoTimer.reset();
                autoState++;
                break;

            case 6:
                if (t > 3) {
                    // stop
                    driveTrain.drive(0, 0, 0);
                    autoState++;
                }
                break;

            default:
                break;
        }
    }

    public void displayHandler() {
        SmartDashboard.putNumber("ActualRPM", shooter.shooterRPM);

        SmartDashboard.putNumber("ActualPower", shooter.actualPower);

        SmartDashboard.putNumber("Angle Encoder(Degrees", deck.deckAngle);

        SmartDashboard.putNumber("Sonar Distance", driveTrain.sonarDistance);

        SmartDashboard.putNumber("Odometer", driveTrain.distanceCounter.get());

        SmartDashboard.putNumber("RawEncoder", deck.angleEncoder.getDistance());

        SmartDashboard.putNumber("TargetRPM", shooter.targetRPM);
    }
}
