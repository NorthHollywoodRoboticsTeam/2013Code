/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

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
    /* Used Ports:
     * Motor ports:
     * 1 weels left,2 weels right,3 loader
     * Relay ports:
     * 1 compressor, 2 shooter
     * Digitalio ports:
     * 1 compressor,2 lifting sensor, 3 loading sensor
     * Solenoid ports:
     * 1 solenoid 1, 2 solenoid 2, 3 loader solenoid 1, 4 loader solenoid 2, 5 lifter double solenoid port 1, 6 lifter double solenoid part 2
     * 
     */
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
    DoubleSolenoid lifterSolenoid = new DoubleSolenoid(5, 6);
    Relay shooter = new Relay(2);
    Servo loader = new Servo(3);
    RobotDrive drive = new RobotDrive(1, 2);
    DigitalInput loadingSensor = new DigitalInput(3);
    DigitalInput liftingSensor = new DigitalInput(2);
    Solenoid loaderSolenoid1 = new Solenoid(3);
    Solenoid loaderSolenoid2 = new Solenoid(4);
    boolean js2ManualMode = false;
    int moveAimer = 0;
    int moveLifter = 0;

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {

        compressor.start();
        //AmingBox box = new AmingBox();
        while (isOperatorControl() && isEnabled()) {
            drive.arcadeDrive(js1);
            js1Lojic();
            js2Logic();
            Timer.delay(.01);
        }
    }
    
    
    boolean[][] pastButtonStates = new boolean[2][10];

    public boolean hasChangedState(int joystickNumber, int buttonNumber) {
        if (pastButtonStates[joystickNumber - 1][buttonNumber - 1] != (joystickNumber == 1 ? js1 : js2).getRawButton(buttonNumber)) {

            pastButtonStates[joystickNumber - 1][buttonNumber - 1] = (joystickNumber == 1 ? js1 : js2).getRawButton(buttonNumber);
            return true;
        } else {
            return false;
        }
    }

    public void extendLifter() {
        lifterSolenoid.set(DoubleSolenoid.Value.kForward);
        Timer.delay(2);
    }

    public void compressLifter() {
        lifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        while (liftingSensor.get()) {}
        lifterSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void moveAimerUp(Solenoid sol1, Solenoid sol2) {
        sol1.set(true);
        sol2.set(true);
    }

    public void moveAimerDown(Solenoid sol1, Solenoid sol2) {
        sol1.set(false);
        sol2.set(false);
    }

    public void holdAimerStill(Solenoid sol1, Solenoid sol2) {
        sol1.set(false);
        sol2.set(true);
    }
    boolean isShooting;

    public void nonBlockingSpit() {
        Thread t = new Thread(new Runnable() {
            public void run() {
                isShooting = true;
                spit();
                isShooting = false;
            }
        });
    }

    public void push() {
        loader.setAngle(45);
        loader.setAngle(0);
    }

    public void feed() {
        moveAimerUp(loaderSolenoid1, loaderSolenoid2);
        while (!loadingSensor.get()) {
        }
        push();
        moveAimerDown(loaderSolenoid1, loaderSolenoid2);
        Timer.delay(.5);
        holdAimerStill(loaderSolenoid1, loaderSolenoid2);
    }

    public void spit() {
        shooter.set(Relay.Value.kReverse);
        feed();
        shooter.set(Relay.Value.kOff);

    }

    public void nonBlockingShoot() {
        Thread t = new Thread(new Runnable() {
            public void run() {
                isShooting = true;
                shoot();
                isShooting = false;
            }
        });
    }

    public void shoot() {
        shooter.set(Relay.Value.kForward);
        feed();
        shooter.set(Relay.Value.kOff);

    }
    public void js2Logic() {
        if (js2.getZ() > .5) {
                js2ManualMode = true;
            } else {
                js2ManualMode = false;
            }


            if (!js2ManualMode) {
                if (js2.getTrigger() && hasChangedState(2, 1)) {
                    if (!isShooting) {
                        nonBlockingShoot();
                    }
                }
                if (js2.getRawButton(5) && hasChangedState(2, 5)) {
                    if (!isShooting) {
                        nonBlockingSpit();
                    }
                }
                if (js2.getRawButton(11)) {
                    extendLifter();
                }
                if (js2.getRawButton(10)) {
                    compressLifter();
                }
                if (js2.getRawButton(8) && hasChangedState(2, 8)) {
                    if (compressor.enabled()) {
                        compressor.stop();
                    } else {
                        compressor.start();
                    }
                }
            } else {
                if (js2.getRawButton(4) && hasChangedState(2, 4)) {
                    push();
                    moveAimerDown(loaderSolenoid1, loaderSolenoid2);
                    Timer.delay(.5);
                    holdAimerStill(loaderSolenoid1, loaderSolenoid2);
                }
                if (js2.getRawButton(6) && hasChangedState(2, 6)) {
                    moveLifter = 20;

                }
                if (js2.getRawButton(7) && hasChangedState(2, 7)) {
                    moveLifter = -20;

                }

                if (moveLifter < 0) {
                    moveAimerDown(loaderSolenoid1, loaderSolenoid2);
                    moveLifter++;
                }
                if (moveLifter > 0) {
                    moveAimerUp(loaderSolenoid1, loaderSolenoid2);
                    moveLifter--;
                } else {
                    holdAimerStill(loaderSolenoid1, loaderSolenoid2);
                }
                
                if (js2.getRawButton(1)) {
                    shooter.set(Relay.Value.kForward);
                } else if (js2.getRawButton(5)) {
                    shooter.set(Relay.Value.kReverse);
                } else {
                    shooter.set(Relay.Value.kOff);
                }
            }
    }
    public void js1Lojic() {
        if (js1.getRawButton(6) && hasChangedState(1, 6)) {
                moveAimer = 20;

            }
            if (js1.getRawButton(7) && hasChangedState(1, 7)) {
                moveAimer = -20;

            }

            if (moveAimer < 0) {
                moveAimerDown(solenoid1, solenoid2);
                moveAimer++;
            }
            if (moveAimer > 0) {
                moveAimerUp(solenoid1, solenoid2);
                moveAimer--;
            } else {
                holdAimerStill(solenoid1, solenoid2);
            }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    }
}
