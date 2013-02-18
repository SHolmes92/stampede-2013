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
import edu.wpi.first.wpilibj.Talon; 
import edu.wpi.first.wpilibj.Encoder; 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */ 
public class RobotTemplate extends IterativeRobot {
    DriverStationLCD driverStation;
    
    RobotDrive drive;
    Joystick leftStick;
    Joystick rightStick;
    Joystick gamepad; 
    Gyro gyro;
    Talon w1;
    Talon w2;
    Talon shooter;
    Talon hopper; 
    Talon angulator;  
    Timer displayTimer; 
    Timer shooterTimer; 

    
    
    double gyroDrift;   // degrees per second;
    double gyroLastRdg;
    double gyroLastTime;
    
    boolean headingLock;    // heading lock mode
    double heading;         // the locked heading (according to gyro)
    double lastAngle;
    double lastAngleTime;
    double shooterRPM; 
    double RPMError; 
    double lastDeckAngle;
    double lastRPM; 
    
    

    
    // sonar
    Ultrasonic rightSonar;
    Ultrasonic leftSonar; 
    boolean sonarLock, sonarLocked;
    
    DigitalInput lightSensor1; 
    DigitalInput lightSensor2; 
    DigitalInput lightSensor3;
    DigitalInput lowerLimit; 
    DigitalInput upperLimit; 
    Counter shooterCounter; 
    Encoder angleEncoder;
    
    boolean shoot, load, trigger;
    boolean moveUp; 
    boolean moveDown;
    double targetRPM; 
    double predictedPower;
    double actualPower; 
    double sonarDistance; 
    double sonarDifference; 
    
    boolean deckTopRequest; 
    boolean deckBottomRequest; 
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        // start!
      //  driverStation = DriverStationLCD.getInstance();
      //  driverStation.println(DriverStationLCD.Line.kUser2, 1, "Robot booting...");
      //  driverStation.updateLCD();
        
      //  System.out.println("Gyrobot booting...");

        // create all objects
       drive = new RobotDrive(1, 2, 3, 4);
       leftStick = new Joystick(1);
       rightStick = new Joystick(2);
       gamepad = new Joystick(3);
       gyro = new Gyro(1);
       w1 = new Talon(5);
       w2 = new Talon(7); 
       angulator = new Talon(2, 1);
       shooter = new Talon(6);
       hopper = new Talon (8); 
       lightSensor1 = new DigitalInput(5);
       lightSensor2 = new DigitalInput(6);
       lightSensor3 = new DigitalInput(7);
       lowerLimit = new DigitalInput(2, 2);
       upperLimit = new DigitalInput(2, 1); 
       shooterCounter = new Counter(lightSensor1); 
       angleEncoder = new Encoder(2, 3, 2, 4);
       angleEncoder.start(); 
       shooterCounter.start(); 
       displayTimer = new Timer();
       shooterTimer = new Timer(); 
       displayTimer.start(); 
       shooterTimer.start(); 
       angleEncoder.setDistancePerPulse(-41.0/26.0 * 4);
       targetRPM = 0; 
       
 
       
       
    
        

        // note: 1 is output (marked INPUT on VEX!!!)
        // note: 2 is input (marked OUTPUT on VEX!!!)
        // note: 3 is output (marked INPUT on VEX!!!) 
        // note: 4 is input (marked OUTPUT on VEX!!!) 
        rightSonar = new Ultrasonic(1, 2);   
        leftSonar = new Ultrasonic(3, 4); 
        

        // start the gyro
        gyro.reset();
        gyro.setSensitivity(1.647 * 0.001);  // VEX gyro sensitivity (in mv/deg/sec)
        
        // start the sonar
        leftSonar.setAutomaticMode(true);
        rightSonar.setAutomaticMode(true); 
        
        // prepare for sample collection for gyro drift correction
        gyroLastRdg = gyro.getAngle();
        gyroLastTime = Timer.getFPGATimestamp();
        gyroDrift = 0;
        
        // initiate the drive system
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        
        shoot = false;
        load = false;
        trigger = false;
    }

    public void disabledInit() {
      //  driverStation.println(DriverStationLCD.Line.kUser2, 1, "Robot disabled.");   
      //  driverStation.updateLCD();
      //  System.out.println("Gyrobot disabled.");
        
        // start gyro compensation routine(note: DO NOT MOVE THE BOT!)
       gyro.reset();
       gyroLastRdg = gyro.getAngle();
       gyroLastTime = Timer.getFPGATimestamp();
       gyroDrift = 0;
    }

    // called periodically during disabled
    public void disabledPeriodic() {
        // we should start reading the gyro here to correct for any drift
        // take a drift reading based on the last 5 seconds (MAY ADJUST!!!)
        double dt = Timer.getFPGATimestamp() - gyroLastTime;
        if(dt >= 10){
             gyroLastTime = Timer.getFPGATimestamp();
        }     
    }
    
   
    public void teleopInit() {
        gyro.reset();
        
      //  driverStation.println(DriverStationLCD.Line.kUser2, 1, "Robot in teleop...");   
      //  driverStation.updateLCD();
    }
    /**
     * This function is called periodically during autonomous
     */
    public void teleopPeriodic() {
        
        
        winchHandler(); 
        shooterHandler(); 
        deckHandler();
        loadHandler(); 
        driveHandler();
        displayHandler(); 
        angleHandler(); 
        shooterEncoder(); 
        sonarHandler(); 
    }
        
    

    /**
     * This function is called periodically during operator control
     */
    public void autonomousPeriodic() {
        // calculate drift-adjusted gyro position
        //double gyroAngle = gyro.getAngle() + gyroDrift * (Timer.getFPGATimestamp() - gyroLastTime);
    } 
     public void deckHandler() { 
     double angulatorPower; 
     angulatorPower = 0;
     moveUp = gamepad.getRawButton(6);
     moveDown = gamepad.getRawButton(8);
 
     if(moveUp && upperLimit.get()){
         deckTopRequest = false; 
         deckBottomRequest = false; 
         angulatorPower = 1; 
     } 
     else if(moveDown && lowerLimit.get()){
         deckTopRequest = false; 
         deckBottomRequest = false; 
         angulatorPower = -1;     
     }
     else if(deckTopRequest && upperLimit.get()){
         angulatorPower = 1; 
     }
     else if(deckBottomRequest && lowerLimit.get()){ 
         angulatorPower = -1; 
     }
     else { 
         deckTopRequest = false; 
         deckBottomRequest = false; 
         angulatorPower = 0; 
     }
       
     angulator.set(angulatorPower); 
     
 }
    
    public void loadHandler() { 
         if(gamepad.getRawButton(5)) {
            if(!shoot && !load && !trigger){
                load = true;
                trigger = true;
            }
        }
        else {
            trigger = false;
        }
        
        if(load){
            // go past the mark
            if(lightSensor2.get()){
                // on the mark - move off of it
                hopper.set(0.35);
            }
            else {
                load = false;
                shoot = true;
            }
        }
        if(shoot) {
            // go to next mark
            if(!lightSensor2.get()){
                // not on the mark - go faster (shooting)
                hopper.set(0.5);
            }
            else {
                // done
                shoot = false;
                load = false;
                hopper.set(0.0);
            }
        }
    }
    
    public void displayHandler() {
        SmartDashboard.putNumber("Avg Sonar Distance", sonarDistance); 
        SmartDashboard.putNumber("RightSonar(Inches)", rightSonar.getRangeInches());
        SmartDashboard.putNumber("LeftSonar(Inches)", leftSonar.getRangeInches()); 
        SmartDashboard.putNumber("SonarDifference", sonarDifference);
        if(gamepad.getRawButton(9)){
        SmartDashboard.putBoolean("LightSensor2", lightSensor2.get());
        SmartDashboard.putBoolean("LightSensor3", lightSensor3.get());
        SmartDashboard.putBoolean("UpperLimit", upperLimit.get());
        SmartDashboard.putBoolean("LowerLimit", lowerLimit.get()); 
        
        SmartDashboard.putNumber("Heading", gyro.getAngle()); 
        SmartDashboard.putNumber("Angle Encoder(Degrees", ((angleEncoder.getDistance() + 16)));
        SmartDashboard.putNumber("ActualRPM", shooterRPM);  
        SmartDashboard.putNumber("TargetRPM", targetRPM);  
        SmartDashboard.putNumber("TargetPower", predictedPower);
        SmartDashboard.putNumber("ActualPower", actualPower); 
        }
    } 
    
    public void gyroHandler(){
    /*    
     * 
     * 
     * double gyroAngle = gyro.getAngle();
       double time = Timer.getFPGATimestamp();
       * 
       *if(leftStick.getRawButton(1)){
            // lock the heading
            if(!headingLock){
                headingLock = true;
                heading = gyroAngle;
            }
        }
        else {
            headingLock = false;
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
    
    */
    }
    public void driveHandler(){

        // get control variables
        double y = 0;   // fwd speed
        double x = 0;   // side motion
        double r = 0;   // rotation

        x = leftStick.getX();
        y = leftStick.getY();
        r = rightStick.getX();
        
        
        if(leftStick.getRawButton(1) && sonarDistance < 30){
            // stop moving forward
            if(y < 0){      
                y = 0.0; 
            }  
            // wall alignment 
            if(sonarDifference > 1.5){
                r = -0.12;
        }
            else if(sonarDifference < -1.5){
                r = 0.12;
            }
            else {
                r = 0.0;
                // final approach 
                if(sonarDistance > 12.0){
                    y = -0.07;
                    deckBottomRequest = true; 
                    targetRPM = 0; 
                    
                }
                else {
                    y = 0.0; 
                }

            } 
        
        } 
        drive.mecanumDrive_Cartesian(x, y, r, 0.0);   
    }
        
        
    
  
    public void sonarHandler() {
        sonarDistance = ((rightSonar.getRangeInches() + leftSonar.getRangeInches())/2); 
        sonarDifference = rightSonar.getRangeInches() - leftSonar.getRangeInches(); 
        
        
    }
    public void hopperHandler(){
         if(gamepad.getRawButton(2)) {
            hopper.set(gamepad.getY());
        }
        else {
            hopper.set(0); 
        }
    }
    public void shooterHandler (){
        if(gamepad.getRawButton(4)){
            targetRPM = (((-gamepad.getY()+1)/2)*1500);
        }
        predictedPower = targetRPM * (0.4/1400.0);
        RPMError = targetRPM - shooterRPM;
        double powerCorrection; 
        powerCorrection = (RPMError * (0.4/1400.0))* 1;
        actualPower = predictedPower + powerCorrection; 
        if(actualPower > 1){
            actualPower = 1; 
        }
        if (actualPower < 0) {
            actualPower = 0; 
        }
        shooter.set(actualPower);
    }
    public void winchHandler(){
       if(gamepad.getRawButton(1)){    
            w1.set(gamepad.getY()*0.6);
        }
        else {
            w1.stopMotor();
        } 
        if(gamepad.getRawButton(3)){
            w2.set(gamepad.getY()*0.6);
        }
        else {
            w2.stopMotor();
        }
    }
    public void angleHandler(){ 
     if(!lowerLimit.get()){
         angleEncoder.reset();
     }
   }
    public void shooterEncoder(){
     
     if(shooterTimer.get() > 1){
         shooterRPM = (shooterCounter.get()/4.0 * 60.0); 
         shooterCounter.reset(); 
         shooterTimer.reset(); 
     }
 }
 
}
    

