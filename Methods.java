package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.media.AudioManager;
import android.media.SoundPool;
import android.media.SoundPool.OnLoadCompleteListener;

/**
 * This awesome program for Singularity Technology
 * was created by Albert on 2/24/2017.
 */
abstract public class Methods extends LinearOpMode {


/*------------------------------------------------------------------
*-------------------------------------------------------------------
* Configuration
*-------------------------------------------------------------------
*-------------------------------------------------------------------*/
    // MATCH CONFIGURATION
    public void matchconfigure(Hardware myrobot, boolean IsTeleOp) {
        // Init Loop
        myrobot.bCompletecolorinit = false;
        myrobot.bCompletedistanceinit = false;
        myrobot.bCompleteinit = false;
        int iLoopCount = 0;

        while (!myrobot.bCompleteinit) {
            // Stops the method if driver presses stop
            if (isStopRequested()) {
                telemetry.addData("STOPPING...", "CONFIGURATION UNFINISHED");
                telemetry.addData("Warning:", "May result in errors");
                telemetry.update();
                return;
            }

            // Displays to show beginning of initloop
            telemetry.addData("Match settings:", "Updating...");

            // Obtains values from gamepad for team color
            if (gamepad2.x) {
                myrobot.bAreWeRed = false;
                myrobot.bCompletecolorinit = true;
            } else if (gamepad2.b) {
                myrobot.bAreWeRed = true;
                myrobot.bCompletecolorinit = true;
            } else {
                myrobot.bCompletecolorinit = false;
            }

            // Obtains values from gamepad for starting location
            if (gamepad2.a) {
                myrobot.bAreWeNear = true;
                myrobot.bCompletedistanceinit = true;
            } else if (gamepad2.y) {
                myrobot.bAreWeNear = false;
                myrobot.bCompletedistanceinit = true;
            } else {
                myrobot.bCompletedistanceinit = false;
            }

            // Adds function for delay (Increases by 1 second with a lockout of 1/2 a second to increase toggle control)
            while (gamepad2.dpad_up && (System.currentTimeMillis() > (Hardware.dInitTimer + 250))) {
                myrobot.lStartDelay = myrobot.lStartDelay + 1000;
                Hardware.dInitTimer = System.currentTimeMillis();
            }
            while (gamepad2.dpad_down && (System.currentTimeMillis() > (Hardware.dInitTimer + 250))) {
                myrobot.lStartDelay = myrobot.lStartDelay - 1000;
                Hardware.dInitTimer = System.currentTimeMillis();
            }
            // Prevents delay from being negative or over 30 seconds
            if (myrobot.lStartDelay > 30000) {
                myrobot.lStartDelay = 30000;
            }
            if (myrobot.lStartDelay < 0) {
                myrobot.lStartDelay = 0;
            }

            // Assigns starting color
            if (myrobot.bAreWeRed) {
                myrobot.sSide = ("Red");
            } else if (!myrobot.bAreWeRed) {
                myrobot.sSide = ("Blue");
            }

            // Assigns starting location
            if (myrobot.bAreWeNear) {
                myrobot.sStartPosition = ("Near");
            } else if (!myrobot.bAreWeNear) {
                myrobot.sStartPosition = ("Far");
            }

            iLoopCount++;

            // Updates telemetry
            telemetry.addData("Start Delay (sec):", (myrobot.lStartDelay / 1000));
            telemetry.addData("Finished sSide init", myrobot.bCompletecolorinit);
            telemetry.addData("Finished distance init", myrobot.bCompletedistanceinit);
            telemetry.addData("Finished init loop", myrobot.bCompleteinit);
            telemetry.update();

            // Determines if init_loop is complete (tests for color in teleop and color + distance in autonomous
            if ((myrobot.bCompletecolorinit && IsTeleOp) || (myrobot.bCompletecolorinit && myrobot.bCompletedistanceinit && !IsTeleOp)) {
                myrobot.bCompleteinit = true;
                break;
            } else {
                myrobot.bCompleteinit = false;
            }
        }

        // Indicates successful configuration
        telemetry.addData("Match settings:", "Set!");
        telemetry.update();
    }


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Autonomous Calculations
    *-------------------------------------------------------------------
    *-------------------------------------------------------------------*/


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Robot Functions
    *-------------------------------------------------------------------
    *-------------------------------------------------------------------*/
    public void Drive_Straight_Nogyro (Hardware myrobot, double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double newspeed;


        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
            myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            myrobot.runtime.reset();
            myrobot.leftBackMotor.setPower(Math.abs(speed));
            myrobot.rightBackMotor.setPower(Math.abs(speed));
            myrobot.leftFrontMotor.setPower(Math.abs(speed));
            myrobot.rightFrontMotor.setPower(Math.abs(speed));

            // Keep looping while there is time left, and all motors are running.
            while ((opModeIsActive()) && (myrobot.runtime.seconds() < timeout) &&
                    (myrobot.leftBackMotor.isBusy() && myrobot.rightFrontMotor.isBusy() &&
                            myrobot.leftFrontMotor.isBusy() && myrobot.rightBackMotor.isBusy())) {

                // Adjust speed to decelerate
                newspeed = deceleration_rate(newLeftBackTarget, myrobot.leftBackMotor.getCurrentPosition(), speed);
                myrobot.leftBackMotor.setPower(Math.abs(newspeed));
                myrobot.rightBackMotor.setPower(Math.abs(newspeed));
                myrobot.leftFrontMotor.setPower(Math.abs(newspeed));
                myrobot.rightFrontMotor.setPower(Math.abs(newspeed));

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                        newLeftBackTarget,
                        newRightBackTarget,
                        newLeftFrontTarget,
                        newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        myrobot.leftBackMotor.getCurrentPosition(),
                        myrobot.rightBackMotor.getCurrentPosition(),
                        myrobot.leftFrontMotor.getCurrentPosition(),
                        myrobot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("Speed (%)", (newspeed * 100));
                telemetry.update();
            }
            // Stop all motion;
            myrobot.leftBackMotor.setPower(0);
            myrobot.rightBackMotor.setPower(0);
            myrobot.leftFrontMotor.setPower(0);
            myrobot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void Drive_Strafe (Hardware myrobot, double speed, double distance, double timeout, String direction) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            // Determine new target position based on direction, and pass to motor controller
            newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            if (direction.equals("Right")) {
                newRightFrontTarget = (-1 * newRightFrontTarget);
                newLeftBackTarget = (-1 * newLeftBackTarget);
            } else if (direction.equals("Left")) {
                newRightBackTarget = (-1 * newRightBackTarget);
                newLeftFrontTarget = (-1 * newLeftFrontTarget);
            }
            myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
            myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            myrobot.runtime.reset();
            myrobot.leftBackMotor.setPower(Math.abs(speed));
            myrobot.rightBackMotor.setPower(Math.abs(speed));
            myrobot.leftFrontMotor.setPower(Math.abs(speed));
            myrobot.rightFrontMotor.setPower(Math.abs(speed));

            // Keep looping while there is time left, and all motors are running.
            while ((opModeIsActive()) && (myrobot.runtime.seconds() < timeout) &&
                    (myrobot.leftBackMotor.isBusy() && myrobot.rightFrontMotor.isBusy() &&
                            myrobot.leftFrontMotor.isBusy() && myrobot.rightBackMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                        newLeftBackTarget,
                        newRightBackTarget,
                        newLeftFrontTarget,
                        newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        myrobot.leftBackMotor.getCurrentPosition(),
                        myrobot.rightBackMotor.getCurrentPosition(),
                        myrobot.leftFrontMotor.getCurrentPosition(),
                        myrobot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            myrobot.leftBackMotor.setPower(0);
            myrobot.rightBackMotor.setPower(0);
            myrobot.leftFrontMotor.setPower(0);
            myrobot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


    public void Drive_Diagonal (Hardware myrobot, double speed, double distance, double timeout, String direction) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double error;
        double angle = myrobot.gyro.getIntegratedZValue();       // Aligns relative to original gyro position
        double steer;
        double leftSpeed;
        double rightSpeed;
        double max;

        if (opModeIsActive()) {
            // Determine new target position
            newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
            newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);

            // Turn On RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time
            myrobot.runtime.reset();

            // Pass target position to motor controllers and start motion
            if (direction.equals("Left/Forward") || direction.equals("Right/Backward")) {
                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

                myrobot.leftBackMotor.setPower(Math.abs(speed));
                myrobot.rightFrontMotor.setPower(Math.abs(speed));
            } else if (direction.equals("Left/Backward") || direction.equals("Right/Forward")) {
                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);

                myrobot.rightBackMotor.setPower(Math.abs(speed));
                myrobot.leftFrontMotor.setPower(Math.abs(speed));
            }

            // Display Telemetry
            // Keep looping while there is time left, and all motors are running.
            if (direction.equals("Left/Forward") || direction.equals("Right/Backward")) {
                while ((opModeIsActive()) && (myrobot.runtime.seconds() < timeout) &&
                        (myrobot.leftBackMotor.isBusy() && myrobot.rightFrontMotor.isBusy())) {
                                        // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d",
                            newLeftBackTarget,
                            newRightFrontTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            myrobot.leftBackMotor.getCurrentPosition(),
                            myrobot.rightFrontMotor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (direction.equals("Left/Backward") || direction.equals("Right/Forward")) {
                while ((opModeIsActive()) && (myrobot.runtime.seconds() < timeout) &&
                        (myrobot.leftFrontMotor.isBusy() && myrobot.rightBackMotor.isBusy())) {
                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d",
                            newRightBackTarget,
                            newLeftFrontTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            myrobot.rightBackMotor.getCurrentPosition(),
                            myrobot.leftFrontMotor.getCurrentPosition());
                    telemetry.update();
                }
            }

            // Stop all motion;
            myrobot.leftBackMotor.setPower(0);
            myrobot.rightBackMotor.setPower(0);
            myrobot.leftFrontMotor.setPower(0);
            myrobot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sleep (100);
    }


    public void Drive_Turn (Hardware myrobot, double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading( myrobot, speed, angle, Hardware.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    public void Align (Hardware myrobot, double speed, double angle, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(myrobot, speed, angle, Hardware.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        myrobot.leftFrontMotor.setPower(0);
        myrobot.leftBackMotor.setPower(0);
        myrobot.rightFrontMotor.setPower(0);
        myrobot.rightBackMotor.setPower(0);
    }


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Supplamentary Functions
    *-------------------------------------------------------------------
    *-------------------------------------------------------------------*/
    public void SnapToWall (Hardware myrobot, double snaptime) {
        // Get z value and initialize targetyaw
        double currentyaw = myrobot.gyro.getIntegratedZValue();
        double targetyaw = 0;

        // Clip values
        while (currentyaw > 180) {
            currentyaw = currentyaw - 360;
        }
        while (currentyaw < -180) {
            currentyaw = currentyaw + 360;
        }

        // Determine target angle based on current heading
        if (-135 <= currentyaw && currentyaw < -45) {
            targetyaw = -90;
        } else if (-45 <= currentyaw && currentyaw < 45) {
            targetyaw = 0;
        } else if (45 <= currentyaw && currentyaw < 135) {
            targetyaw = 90;
        } else if (135 <= currentyaw) {
            targetyaw = 180;
        } else if (currentyaw < -135) {
            targetyaw = -180;
        }

        // Align to target yaw
        Align(myrobot, Hardware.dALIGN_SPEED, targetyaw, snaptime);
    }

    public void calibrate_gyro (Hardware myrobot) {
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        myrobot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && myrobot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
    }


    public void GetLightLevel (Hardware myrobot) {
        int iRetries = 0;
        telemetry.addData("Getting light level", "...");
        telemetry.update();

        //Gets light level and saves it to a variable in the hardware class
        if (myrobot.ODS_L.getLightDetected() < 0.3 && myrobot.ODS_L.getLightDetected() > 0.05) {

            Hardware.dFloor_Light_Level = myrobot.ODS_L.getLightDetected();
        } else {

            while (myrobot.ODS_L.getLightDetected() > 0.3 && iRetries < 15 || myrobot.ODS_L.getLightDetected() < 0.05 && iRetries < 15) {
                sleep(200);
                iRetries += 1;
            }

            Hardware.dFloor_Light_Level = myrobot.ODS_L.getLightDetected();
        }
        // Update White Threshold
        Hardware.dWHITE_THRESHOLD = Hardware.dFloor_Light_Level + Hardware.dWhite_Difference;

        //Adds data to telemetry
        telemetry.addData("Light Level is", Hardware.dFloor_Light_Level);
        telemetry.update();
    }


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * PID Functions
    *-------------------------------------------------------------------
    *-------------------------------------------------------------------*/

    boolean onHeading(Hardware myrobot, double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle, myrobot);

        if (Math.abs(error) <= Hardware.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        myrobot.leftFrontMotor.setPower(leftSpeed);
        myrobot.leftBackMotor.setPower(leftSpeed);
        myrobot.rightFrontMotor.setPower(rightSpeed);
        myrobot.rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle, Hardware myrobot) {
        double robotError;

        robotError = targetAngle - myrobot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double deceleration_rate (double target, double current, double originalspeed) {
        double remainder;
        double speedfactor;
        double newspeed;
        // Calculate remaining ticks
        remainder = target - current;

        // obtain positive value
        remainder = Math.abs(remainder);

        // Divide remainder by deceleration threshold
        speedfactor = (remainder/1200);

        // Clip speedfactor to ensure values between 20% and 100%
        speedfactor = Range.clip(speedfactor, 0.2, 1);

        // Multiply original speed by speedfactor
        newspeed = (originalspeed * speedfactor);
        return newspeed;
    }
	


/*
    public void playsound(int soundfile) {
        final SoundPool mySound;
        final int beepID;
        final boolean[] loaded = new boolean[1];
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, soundfile, 1); // PSM

        mySound.setOnLoadCompleteListener(new OnLoadCompleteListener() {
            @Override
            public void onLoadComplete(SoundPool soundPool, int mySoundId, int status) {
                loaded[0] = true;
                try {
                    mySound.play(beepID, 1, 1, 1, 0, 1);
                } catch (NullPointerException e) {
                    telemetry.addData("Error:", "No Sound File");
                    telemetry.update();
                }
            }
        });
    }
    */

}
