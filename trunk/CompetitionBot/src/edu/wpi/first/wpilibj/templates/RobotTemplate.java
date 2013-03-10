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
import edu.wpi.first.wpilibj.Servo; 
import edu.wpi.first.wpilibj.Relay; 

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
    Servo pusher, tapper; 
    Relay LightRelay; 
     

    
    
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
    
    boolean firingPosition; 
    

    
    // sonar
    Ultrasonic rightSonar;
    Ultrasonic leftSonar; 
    boolean sonarLock, sonarLocked;
    
    DigitalInput fresbeeSensor; 
    DigitalInput mixerSensor; 
    DigitalInput shooterWheelSensor;
    DigitalInput lowerLimit; 
    DigitalInput upperLimit; 
    Counter shooterCounter; 
    Encoder angleEncoder;
    Encoder rightWinchEncoder; 
    Encoder leftWinchEncoder; 
    
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
    
    double y = 0;   // fwd speed
    double x = 0;   // side motion
    double r = 0;
    
    // launcher support variables
    boolean[] launcherSlots;   // 0 = loading slot, 3 = chamber, false = empty
    boolean launcherTurning;
    boolean launcherSettling;
    boolean launcherLoading;
    boolean launcherShooting;
    boolean launcherPausing;
    boolean launcherPastMark;
    boolean discInFeeder;
    boolean fresbeeDetected;
    Timer shotTimer, turnTimer, settlingTimer, feederTimer;

    
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
       pusher = new Servo(9); 
       tapper = new Servo(10);
       pusherOut();
       tapperUp();
       fresbeeSensor = new DigitalInput(7);
       mixerSensor = new DigitalInput(6);
       shooterWheelSensor = new DigitalInput(5);
       lowerLimit = new DigitalInput(2, 2);
       upperLimit = new DigitalInput(2, 1); 
       shooterCounter = new Counter(shooterWheelSensor); 
       angleEncoder = new Encoder(2, 3, 2, 4);
       rightWinchEncoder = new Encoder(1, 8, 1, 9);
       leftWinchEncoder = new Encoder(1, 10, 1, 11); 
       angleEncoder.start(); 
       shooterCounter.start(); 
       displayTimer = new Timer();       
       shooterTimer = new Timer(); 
       displayTimer.start(); 
       shooterTimer.start(); 
       rightWinchEncoder.start(); 
       leftWinchEncoder.start(); 
       angleEncoder.setDistancePerPulse(-41.0/26.0 * 4);
       targetRPM = 0; 
       LightRelay = new Relay(2, 1, Relay.Direction.kForward); 
       
       // initial launcher configuration
       launcherSlots = new boolean[4];
       launcherSlots[0] = false;
       launcherSlots[1] = true;
       launcherSlots[2] = true;
       launcherSlots[3] = true;
       launcherTurning = false;
       launcherSettling = false;
       launcherShooting = false;
       launcherLoading = false;
       launcherPausing = false;
       launcherPastMark = false;
       discInFeeder = false;
       fresbeeDetected = false;
       turnTimer = new Timer();
       settlingTimer = new Timer();
       shotTimer = new Timer();
       feederTimer = new Timer();
       
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
        
       launcherSlots[0] = true;
       launcherSlots[1] = true;
       launcherSlots[2] = true;
       launcherSlots[3] = true;
       
       launcherTurning = false;
       launcherSettling = false;
       launcherShooting = false;
       launcherLoading = false;
       launcherPausing = false;
       launcherPastMark = false;
       discInFeeder = launcherSlots[0];
       fresbeeDetected = false;
       LightRelay.set(Relay.Value.kOn);
        
        pusherOut();
        tapperDown();
 
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
        launcherHandler();
        // loadHandler(); 
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
     firingPosition = rightStick.getRawButton(3); 
 
     if(moveUp && upperLimit.get()){
         deckTopRequest = false; 
         deckBottomRequest = false; 
         firingPosition = false; 
         angulatorPower = 1; 
     } 
     else if(moveDown && lowerLimit.get()){
         deckTopRequest = false; 
         deckBottomRequest = false; 
         firingPosition = false; 
         angulatorPower = -1;     
     }
     else if(deckTopRequest && upperLimit.get()){
         angulatorPower = 1; 
     }
     else if(deckBottomRequest && lowerLimit.get()){ 
         angulatorPower = -1;
     }
     else if(firingPosition && (angleEncoder.getDistance() + 16) > 40){
         angulatorPower = -1; 
     }
     else if(firingPosition && (angleEncoder.getDistance() + 16) < 35){
         angulatorPower = 1; 
     }
     else { 
         deckTopRequest = false; 
         deckBottomRequest = false; 
         firingPosition = false; 
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
            if(fresbeeSensor.get()){
                // on the mark - move off of it
                hopper.set(0.55);
            }
            else {
                load = false;
                shoot = true;
            }
        }
        if(shoot) {
            // go to next mark
            if(!mixerSensor.get()){
                // not on the mark - go faster (shooting)
                hopper.set(0.6);
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
        SmartDashboard.putNumber("Right Winch Encoder(Inches)", rightWinchEncoder.getDistance());
        SmartDashboard.putNumber("Left Winch Encoder(Inches)", leftWinchEncoder.getDistance()); 
       
        if(gamepad.getRawButton(9)){
        SmartDashboard.putBoolean("fresbeeSensor", fresbeeSensor.get());
        SmartDashboard.putBoolean("mixerSensor", mixerSensor.get());
        SmartDashboard.putBoolean("shooterWheelSensor", shooterWheelSensor.get());
        
        SmartDashboard.putNumber("Avg Sonar Distance", sonarDistance); 
        SmartDashboard.putNumber("RightSonar(Inches)", rightSonar.getRangeInches());
        SmartDashboard.putNumber("LeftSonar(Inches)", leftSonar.getRangeInches()); 
        SmartDashboard.putNumber("SonarDifference", sonarDifference);
        SmartDashboard.putNumber("Heading", gyro.getAngle()); 
        
        SmartDashboard.putNumber("ActualRPM", shooterRPM);  
        SmartDashboard.putNumber("TargetRPM", targetRPM);  
        SmartDashboard.putNumber("TargetPower", predictedPower);
        SmartDashboard.putNumber("ActualPower", actualPower);  
        
        SmartDashboard.putNumber("Angle Encoder(Degrees", ((angleEncoder.getDistance() + 16)));
        SmartDashboard.putBoolean("UpperLimit", upperLimit.get());
        SmartDashboard.putBoolean("LowerLimit", lowerLimit.get()); 
        }
    } 
    
    public void driveHandler(){
        
       x = leftStick.getX();
       y = leftStick.getY();
       r = rightStick.getX();
      
      double gyroAngle = gyro.getAngle();
      double time = Timer.getFPGATimestamp();
       
       if(rightStick.getRawButton(1)){
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
        if(headingLock){
            drive.mecanumDrive_Cartesian(x, y, r, gyroAngle);
        }
        else {
            drive.mecanumDrive_Cartesian(x, y, r, 0.0);
        }
        
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
        
        // special case for low power testing
        // if(targetRPM < 750){
        if(true){
            actualPower = targetRPM / 1500;
        }
        
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
 
    public void startTurning()
    {
        turnTimer.reset();
        turnTimer.start();
        launcherTurning = true;
    }
    
    private void tapperUp()
    {
        tapper.set(1.0);
    }
    private void tapperDown()
    {
        tapper.set(0.64);
    }
    private void pusherOut()
    {
        pusher.set(0.0);
    }
    private void pusherIn()
    {
        pusher.set(0.4);
    }
    
    
    public void launcherHandler()
    {
        // handle user interface
        if(gamepad.getRawButton(5) && !(launcherShooting || launcherLoading || launcherPausing)) {
            launcherLoading = true;
            launcherShooting = true;
        }

        // handle disc detector
        if(fresbeeSensor.get()){
            if(!fresbeeDetected){
                fresbeeDetected = true;
                feederTimer.reset();
                feederTimer.start();
            }
            else {
                if(!discInFeeder && feederTimer.get() > 0.25){
                    feederTimer.stop();
                    feederTimer.reset();
                    discInFeeder = true;
                }
            }
        }
        else {
            discInFeeder = false;
            fresbeeDetected = false;
        }
        
        // mark feeder slot as occupied
        if(discInFeeder) {
            launcherSlots[0] = true;
        }

        // turn if chamber empty or slot 2 empty
        if(launcherSlots[0] && (!launcherSlots[3] || !launcherSlots[2]) && !launcherTurning && !launcherSettling && !launcherLoading && !launcherShooting){
        // if(gamepad.getRawButton(10)){
            startTurning(); // DO NOTY JUST SET launcherTurning to true
        }
        
        // now handle the launcher state machine
        if(launcherTurning){
            if(turnTimer.get() < 0.10){
                launcherPastMark = false;
                // lift tapper servo
                tapperUp();
            }
            else if(turnTimer.get() < 1.0){
                // start turning
                hopper.set(0.75);
                turnTimer.stop();
            }
            else if(turnTimer.get() < 3.0){
                
            }
            
            if(!launcherPastMark){
                // spinning - wait to hit the mark...
               if(!mixerSensor.get()){
                   launcherPastMark = true;
               }
            }
            else {
                // stop when past the mark
                if(mixerSensor.get()){
                    int i;
                    hopper.set(0.0);
                    launcherTurning = false;
                    launcherSettling = true;
                    settlingTimer.reset();
                    settlingTimer.start();
                    // lower tapper servo
                    tapperDown();

                    // done turning - shift slots
                    // note: we do NOT shift emptiness into slot 3, only a fresbee (if there was one in slot 2!)
                    // chamber can only be loaded, gets emptied via a shot
                    if(launcherSlots[2]) { launcherSlots[3] = true; }
                    launcherSlots[2] = launcherSlots[1];
                    launcherSlots[1] = launcherSlots[0];
                    launcherSlots[0] = false;   // feeder slot
                }
            }
        }
        else if(launcherSettling){
            if(settlingTimer.get() > 0.3){
                settlingTimer.stop();
                launcherSettling = false;
            }
        }
        
        else if(launcherLoading){
            // turn if disc available but not in the chamber
            if((!launcherSlots[3]) && (launcherSlots[2] || launcherSlots[1] || launcherSlots[0])){
                startTurning();
            }
            else {
                // loading done
                launcherLoading = false;
                // check if load successful - if not cancel a shoot
                if((!launcherSlots[3]) && launcherShooting) {
                    launcherShooting = false;
                }
            }
        }
        else if(launcherShooting){
            // set the servo
            pusherIn();
            
            // start the shotTimer
            shotTimer.reset();
            shotTimer.start();
            launcherShooting = false;
            launcherPausing = true;
        }
        else if(launcherPausing){
            // may do some more here to move the second shooter servo
            if(shotTimer.get() > 0.6){
                // shot complete
                shotTimer.stop();
                launcherPausing = false;
                launcherSlots[3] = false;    // disc out!
                pusherOut();
            }
        }
        else {
               // idle - withdraw servo
               pusherOut();

               // if disc in slot 0 and have room - turn
               
        }
    }
}

    

