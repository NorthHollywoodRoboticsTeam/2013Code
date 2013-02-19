/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        
    }
    
    
    Joystick js1 = new Joystick(1);
    Joystick js2 = new Joystick(2);
    Compressor compressor = new Compressor(1, 1);
    /**
     * Note that solenoid 1 is the solenoid closer to the compressor
     */
    Solenoid solenoid1 = new Solenoid(1);
    /**
     * Note that solenoid 2 is the solenoid farther from the compressor
     */
    Solenoid solenoid2 = new Solenoid(2);
    Relay shooter = new Relay(2);
    RobotDrive drive = new RobotDrive(1, 2);
    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        boolean shooterOn = false;
        boolean trigerPushed = false;
        //AmingBox box = new AmingBox();
        while (isOperatorControl() && isEnabled()) {
            drive.arcadeDrive(js1);
            if (js1.getTrigger()) {
                if (!trigerPushed) {
                    if (shooterOn) {
                        shooter.set(Relay.Value.kOff);
                        shooterOn = false;
                    } else {
                        shooter.set(Relay.Value.kForward);
                        shooterOn = true;
                    }
                    trigerPushed = true;
                }
            } else {
                trigerPushed = false;

            }
            
            if (js1.getRawButton(6)) {
                moveAimerUp();
            }
            if (js1.getRawButton(7)) {
                moveAimerDown();
            }
            if (!js1.getRawButton(7) && !js1.getRawButton(6)) {
                holdAimerStill();
            }
        }
    }
    
    public void moveAimerUp() {
        solenoid1.set(true);
        solenoid2.set(true);
    }
    
    public void moveAimerDown() {
        solenoid1.set(false);
        solenoid2.set(false);
    }
    
    public void holdAimerStill() {
        solenoid1.set(false);
        solenoid2.set(true);
    }
    
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
