package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This awesome program for Singularity Technology
 * was created by Albert on 12/20/2016.
 */

public class Hardware {


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Hardware Initializations
    *-------------------------------------------------------------------
    *-----------------------------------------------------------------*/
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public Servo Chicken;
    public DcMotor CapLift;
    public OpticalDistanceSensor ODS_L;
    public ColorSensor colorsensor;
    public UltrasonicSensor ultrasonicsensor;
    public Servo CapServo;
    public Servo CapRelease;
    public ModernRoboticsI2cGyro gyro;


    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Initial Variables
    *-------------------------------------------------------------------
    *-----------------------------------------------------------------*/
    // Variables for Driving
    boolean bSlowmode;
    boolean bControl;
    boolean bCurrentControl;
    String sControl;
    boolean bControltoggle;

    // Variables for EncoderDrive
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP, CHANGE AS NECESSARY
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference, CHANGE AS NECESSARY
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double dDRIVE_SPEED = 0.78;
    static final double dTURN_SPEED = 0.20;
    static final double dAPPROACH_SPEED = 0.2;
    static final double dTRANSLATE_SPEED = 0.2;
    static final double dALIGN_SPEED = 0.1;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // Variables for Match Configuration
    boolean bDetermined_Color_Success;
    boolean bAlignstatus;
    long lTimeInitial;
    long lTimeDuration = 1000;
    int iTeamColor;
    int iOppColor;

    // Variables for initial position of beacon-presser
    public final static double dChickenhome = 0.35;
    public final static double dChickenleft = 0.09;
    public final static double dChickenright = 0.65;
    public static double dChickenposition = dChickenhome;
    public static String sChicken_Position;
    public static int iMin_Servo = 0;
    public static int iMax_Servo = 1;

    // Variables for initial position and rate of Cap Lifter
    public static double dCaplifthome = 0.0;

    // Variables for cap release servo
    public static double dReleasehome = 0.0;
    public static double dReleasefinish = 1.0;

    // Variables for Cap Servo
    public static double dCapServoRest = 0.0;
    public static double dCapServoSpeed = 0.03;
    public double dCapServoPose;

    // Variable for a threshold (deadband) to eliminate erroneous gamepad values
    static double dDeadzone = 0.02;

    // Variable for sensitivity value (between 0 and 1)
    static double sensitivity = .95;

    // Variable for set sensor values
    public static double dFloor_Light_Level = 0;
    public static double dWhite_Difference = 0.05; // Variable to measure change in light value to detect line
    public static double dWHITE_THRESHOLD = 0.20;  // spans between 0.0-1.0
    static final double dDISTANCE_THRESHOLD = 15;
    static final double dCOLOR_THRESHOLD = 2;

    // Variable for threshold for triggers on controllers
    static final float fHIT_THRESHOLD = (float) 0.8;

    // Variables for Match Configuration
    boolean bAreWeRed;
    boolean bAreWeNear;
    boolean bCompletecolorinit;
    boolean bCompletedistanceinit;
    boolean bCompleteinit;
    String sSide;
    String sStartPosition;
    long lStartDelay = 0;
    static double dInitTimer;

    // Variables for Autonomous
    static double dDistance1;                   // Distance for Drive 1

    static double dTurn1;                       // Angle for Turn 1

    static double dDiag1;                       // Distance for Diag 1
    static String sDiag_Direction1;             // Direction for diagonal 1 (Left or Right)

    static double dDiag2;                       // Distance for Diag 2
    static String sDiag_Direction2;             // Direction for diagonal 2 (Left or Right)

    static double dStrafe1;                     // Distance for Strafe 1
    static String sStrafe_Direction1;           // Direction for Strafe 1 (Left or Right)

    static double dBackangle;                   // Angle to align to when retreating from wall

    static double dDistance2;



    // Variables to get to Ball
    static double dFar_Start_Ball = 60;          // 60 Inches to Ball
    static double dNear_Start_Ball = 45;         // 45 Inches to Ball

    // Variables for Beacon
    static double dFar_to_Stage = 42;           // 42 inches from far position to staging area
    static double dNear_to_Stage = 7.5;           // 6 inches from near position to staging area
    static double dDiagonal_Start = 47;         // Initial inches to drive diagonally during first move
    static double dBlue_Stage_Factor = 20;      // Inches to align to left of beacon (for blue; red is already on left)
    static double dBackward_diag = -78;
    static double dTurn_to_wall = -90;
    static double dBeac_to_Beac_RED = 42;       // 25 inches traslateing to other beacon
    static double dBeac_to_Beac_BLUE = 49;         // Adds blue factor to translate to get robot on left side of blue far beacon

    static double dBack_From_Wall = -7;        // 7 Inches for Driving back from wall

    // Variables for Ball to Ramp
    static double dStart_ball_Near = 50;
    static double dStart_ball_Far = 15;
    static double dStart_diagball_Far = 60;
    static double dTurnToRamp = 120;              // 120 Degree Turn
    static double dBall_Ramp = 56;              // 56 Inches from Ball to Ramp

    // Variables for Ramp
    static double dFar_Start_Ramp = 58;         // 58 Inches to Ramp (Far)
    static double dNear_Start_Ramp = 25;        // 25 Inches to Ramp (Near)
    static double dTurn_46 = 14.5;              // 45 Degree Turn
    static double dTurn_33 = 10.5;              // 33 Degree Turn
    static double dRamp_Park = 35;              // 35 Inches onto Ramp

    /*------------------------------------------------------------------
    *-------------------------------------------------------------------
    * Initialization
    *-------------------------------------------------------------------
    *-----------------------------------------------------------------*/
    // Initializes hardware
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Initializes telemetry
    Telemetry mytelemetry = null;

    public Hardware(Telemetry telemetry) {
        this.mytelemetry = telemetry;
    }

    // INITIALIZATION METHOD
    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;

        // Get references from hardware map
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        leftBackMotor = hwMap.dcMotor.get("leftBack");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        rightBackMotor = hwMap.dcMotor.get("rightBack");
        Chicken = hwMap.servo.get("Chicken");
        CapLift = hwMap.dcMotor.get("Cap");
        ODS_L = hwMap.opticalDistanceSensor.get("ODS_L");
        colorsensor = hwMap.colorSensor.get("colorsensor");
        colorsensor.setI2cAddress(I2cAddr.create7bit(0x26));
        ultrasonicsensor = hwMap.ultrasonicSensor.get("ultrasonicsensor");
        CapServo = hwMap.servo.get("CapServo");
        CapRelease = hwMap.servo.get("CapRelease");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        // Reverse left motors and Cap Lifter
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        CapLift.setDirection(DcMotor.Direction.REVERSE);

        // Reset Encoders to 0
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CapLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets to RUN_USING_ENCODER
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CapLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Sets max ticks for encoders (affects speed proportionally)
        int tick = 4000;
        leftFrontMotor.setMaxSpeed(tick);
        leftBackMotor.setMaxSpeed(tick);
        rightFrontMotor.setMaxSpeed(tick);
        rightBackMotor.setMaxSpeed(tick);

        // Sets limits for servo
        Chicken.scaleRange(iMin_Servo, iMax_Servo);
        CapServo.scaleRange(iMin_Servo, iMax_Servo);

        // Turns LED off Colorsensor
        colorsensor.enableLed(false);

        // Sets initial position of the beacon-presser
        dChickenposition = dChickenhome;
        Chicken.setPosition(dChickenposition);

        // Sets initial power of Cap lifter
        CapLift.setPower(dCaplifthome);

        // Sets Initial time for alignment lockout (So we can run it as soon as Teleop begins)
        lTimeInitial = (System.currentTimeMillis() - 5000);

        // Sets Initial time for delay lockout (Increases control of delay toggle)
        dInitTimer = 0;

        // Sets Initial Determined Color to false
        bDetermined_Color_Success = false;

        // Sets Initial Slow mode of false
        bSlowmode = false;

        // Sets Initial position of Cap Servo
        CapServo.setPosition(dCapServoRest);

        // Sets Initial position of Cap Release;
        CapRelease.setPosition(dReleasehome);

        // Sets Control scheme to normal
        bControl = true;
        bCurrentControl = true;

        // Indicates successful initialization
        mytelemetry.addData("Hardware", "Initialized!");
        mytelemetry.update();
    }
}
