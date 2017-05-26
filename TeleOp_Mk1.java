package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 * This awesome program for Singularity Technology
 * was created by Albert on 12/20/2016.
 */
@TeleOp(name = "TeleopMk8")
public class TeleOp_Mk1 extends Methods {
    Hardware robot = new Hardware(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Init phase
        robot.init(hardwareMap);
        calibrate_gyro(robot);

        matchconfigure(robot, true);

        // Adds Telemetry to indicate successful initialization
        telemetry.addData("READY TO PLAY", "!");
        telemetry.addData("Color =", robot.sSide);
        telemetry.update();

        // Driver Press Play
        waitForStart();

        // Main Loop
        while (opModeIsActive()) {


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Beacon Servo
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            // Clips dChickenposition to ensure values will not exceed servo's capabilities
            Hardware.dChickenposition = Range.clip(Hardware.dChickenposition, Hardware.iMin_Servo, Hardware.iMax_Servo);

            // Assigns left, right, and home positions to gamepad
            if (gamepad1.x) {
                Hardware.dChickenposition = (Hardware.dChickenleft);
            } else if (gamepad1.b) {
                Hardware.dChickenposition = (Hardware.dChickenright);
            } else if (gamepad1.a) {
                Hardware.dChickenposition = Hardware.dChickenhome;
            }

            // Sets string value based on servo position
            if (Hardware.dChickenposition == Hardware.dChickenleft) {
                Hardware.sChicken_Position = "Left";
            } else if (Hardware.dChickenposition == Hardware.dChickenright) {
                Hardware.sChicken_Position = "Right";
            } else if (Hardware.dChickenposition == Hardware.dChickenhome) {
                Hardware.sChicken_Position = "Middle";
            } else {
                Hardware.sChicken_Position = String.valueOf(Hardware.dChickenposition);
            }

            // Assigns the servo to the value input by controller
            robot.Chicken.setPosition(Hardware.dChickenposition);


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * CAP Lifting
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            // Obtains value from gamepad
            double raw_Cap_power = -gamepad2.left_stick_y;

            // Clips values
            raw_Cap_power = Range.clip(raw_Cap_power, -1, 1);

            // Use deadband to eliminate erroneous joystick values
            if (Math.abs(raw_Cap_power) < Hardware.dDeadzone) {
                raw_Cap_power = 0;
            }

            // Uses algorithm to increase sensitivity in joysticks (creates integrated value)
            double intg_Cap_power = Hardware.sensitivity * (raw_Cap_power * raw_Cap_power * raw_Cap_power) + (1 - Hardware.sensitivity) * raw_Cap_power;

            // Assigns motor to integrated Cap power value
            robot.CapLift.setPower(intg_Cap_power);
            

        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * CAP servo and release
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            // Move Cap Release servo to release cap lifter
            if (gamepad2.y) {
                robot.dCapServoPose = (robot.dCapServoPose + Hardware.dCapServoSpeed);
            } else if (gamepad2.a) {
                robot.dCapServoPose = (robot.dCapServoPose - Hardware.dCapServoSpeed);
            }

            // Clip values
            robot.dCapServoPose = Range.clip(robot.dCapServoPose, Hardware.iMin_Servo, Hardware.iMax_Servo);

            // Set servo to position
            robot.CapServo.setPosition(robot.dCapServoPose);

            // Release Cap ball lifter (cannot undo)
            if (gamepad1.y) {
                robot.CapRelease.setPosition(Hardware.dReleasefinish);
            }


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Hit Beacon (Auto)
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            // Creates Alignment lockout
            if (System.currentTimeMillis() < robot.lTimeInitial + robot.lTimeDuration) {
                robot.bAlignstatus = false;
            } else if (System.currentTimeMillis() >= (robot.lTimeInitial + robot.lTimeDuration)) {
                robot.bAlignstatus = true;
            }

            // Calls for alignment method
            while (gamepad1.left_bumper && robot.bAlignstatus) {
                // Calls for method 'alignment'
                //Hitbeacon(robot, Hardware.dAPPROACH_SPEED, Hardware.dTRANSLATE_SPEED, true);
            }


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Driving (CALCULATIONS)
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            double raw_dX1;                         // Rotate
            double raw_dY1;                         // Forward/Backward
            double raw_dX2;                         // Translate

            // Obtain values from gamepad (2 x axis and 1 y axis)
            raw_dX1 = gamepad1.left_stick_x;
            raw_dY1 = -(gamepad1.left_stick_y);
            raw_dX2 = gamepad1.right_stick_x;

            //Inverts forward / backward / translate on left joystick if left bumper is pressed
            if (System.currentTimeMillis() < robot.lTimeInitial + robot.lTimeDuration) {
                robot.bControltoggle = false;
            } else if (System.currentTimeMillis() >= (robot.lTimeInitial + robot.lTimeDuration)) {
                robot.bControltoggle = true;
            }

            if (gamepad1.left_trigger > Hardware.fHIT_THRESHOLD && robot.bControltoggle) {
                robot.bControl = !robot.bCurrentControl;
                robot.bCurrentControl = robot.bControl;
                robot.lTimeInitial = System.currentTimeMillis();
            }
            if (robot.bControl) {
                robot.sControl = "Regular";
            } else if (!robot.bControl) {
                robot.sControl = "Inverted";
                raw_dY1 = (-1 * raw_dY1);
                raw_dX2 = (-1 * raw_dX2);
            }

            // Clip Gamepad Values not to exceed -1/+1
            raw_dX1 = Range.clip(raw_dX1, -1, 1);
            raw_dY1 = Range.clip(raw_dY1, -1, 1);
            raw_dX2 = Range.clip(raw_dX2, -1, 1);

            // Use deadband to eliminate erroneous joystick values
            if (Math.abs(raw_dX1) < Hardware.dDeadzone) {
                raw_dX1 = 0;
            }
            if (Math.abs(raw_dY1) < Hardware.dDeadzone) {
                raw_dY1 = 0;
            }
            if (Math.abs(raw_dX2) < Hardware.dDeadzone) {
                raw_dX2 = 0;
            }

            // Uses algorithm to increase sensitivity in joysticks (creates integrated variables)
            double intg_dX1 = Hardware.sensitivity * (raw_dX1 * raw_dX1 * raw_dX1) + (1 - Hardware.sensitivity) * raw_dX1;
            double intg_dY1 = Hardware.sensitivity * (raw_dY1 * raw_dY1 * raw_dY1) + (1 - Hardware.sensitivity) * raw_dY1;
            double intg_dX2 = Hardware.sensitivity * (raw_dX2 * raw_dX2 * raw_dX2) + (1 - Hardware.sensitivity) * raw_dX2;

            // Calculate power values based on gamepad values
            double dFrontLeft = intg_dY1 + intg_dX1 + intg_dX2;
            double dRearLeft = intg_dY1 + intg_dX1 - intg_dX2;
            double dFrontRight = intg_dY1 - intg_dX1 - intg_dX2;
            double dRearRight = intg_dY1 - intg_dX1 + intg_dX2;

            // Clip Power Values not to exceed -1/+1
            dFrontLeft = Range.clip(dFrontLeft, -1, 1);
            dRearLeft = Range.clip(dRearLeft, -1, 1);
            dFrontRight = Range.clip(dFrontRight, -1, 1);
            dRearRight = Range.clip(dRearRight, -1, 1);


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Driving (Slow mode)
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/

            // Turns slow mode on if bumper is pressed or cap lift is moving
            robot.bSlowmode = (gamepad1.right_bumper) || (Math.abs(intg_Cap_power) > 0.05);

            // Adds a slow speed function for driving
            if (robot.bSlowmode) {
                dFrontLeft = (dFrontLeft / 4);
                dRearLeft = (dRearLeft / 4);
                dFrontRight = (dFrontRight / 4);
                dRearRight = (dRearRight / 4);
            }


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Driving (IMPLEMENTATIONS * Performs driving function*)
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/
            // Performs Driving Function
            robot.leftFrontMotor.setPower(dFrontLeft);
            robot.leftBackMotor.setPower(dRearLeft);
            robot.rightFrontMotor.setPower(dFrontRight);
            robot.rightBackMotor.setPower(dRearRight);


        /*------------------------------------------------------------------
        *-------------------------------------------------------------------
        * Telemetry
        *-------------------------------------------------------------------
        *-------------------------------------------------------------------*/

            // Adds telemetry to display
            telemetry.addData("Control Scheme:", robot.sControl);
            telemetry.addData("Angle", robot.gyro.getIntegratedZValue());
            telemetry.addData("Slow mode:", robot.bSlowmode);
            telemetry.addData("Can run Hit beacon (Auto):", robot.bAlignstatus);
            telemetry.addData("Found Beacon Color (Auto)?", robot.bDetermined_Color_Success);
            telemetry.addData("Beacon Servo:", Hardware.sChicken_Position);
            telemetry.addData("CAP Lift Power:", intg_Cap_power);
            telemetry.addData("CAP Gripper", robot.dCapServoPose);
            telemetry.addData("Light Level:", robot.ODS_L.getLightDetected());
            telemetry.addData("Red Light:", robot.colorsensor.red());
            telemetry.addData("Blue Light:", robot.colorsensor.blue());
            telemetry.addData("Distance:", robot.ultrasonicsensor.getUltrasonicLevel());

            // Update telemetry
            telemetry.update();
        }
    }
}

