package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDriverControlled
{
    /* Public OpMode members. */
    public DcMotor leftfrontMotor = null;
    public DcMotor leftbackMotor = null;
    public DcMotor rightfrontMotor = null;
    public DcMotor rightbackMotor = null;
//    public DcMotor balllauncherMotor1 = null;
//    public DcMotor balllauncherMotor2 = null;
//    public DcMotor  armMotor    = null;
//    public Servo    leftbeaconPusher    = null;
//    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDriverControlled(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftfrontMotor = hwMap.dcMotor.get("leftfrontMotor");
        leftbackMotor = hwMap.dcMotor.get("leftbackMotor");
        rightfrontMotor = hwMap.dcMotor.get("rightfrontMotor");
        rightbackMotor = hwMap.dcMotor.get("rightbackMotor");
//        balllauncherMotor1 = hwMap.dcMotor.get ("balllauncherMotor1");
//        balllauncherMotor2 = hwMap.dcMotor.get ("balllauncherMotor2");
//        armMotor    = hwMap.dcMotor.get("left_arm");
        leftfrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
//        balllauncherMotor1.setDirection(DcMotor.Direction.REVERSE);
//        balllauncherMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftfrontMotor.setPower(0);
        rightfrontMotor.setPower(0);
        leftbackMotor.setPower(0);
        rightbackMotor.setPower(0);
//        balllauncherMotor1.setPower(0);
 //       balllauncherMotor2.setPower(0);
//        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        balllauncherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        balllauncherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
  //      leftbeaconPusher = hwMap.servo.get("left_beacon");
//        rightClaw = hwMap.servo.get("right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

