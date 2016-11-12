/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR ODS", group = "Sensor")
@Disabled
public class SensorMROpticalDistance extends LinearOpMode {

  OpticalDistanceSensor odsSensor;  // Hardware Device Object
  static final double     stopMotor    = 0 ;
  static final double     goSlow    = .07 ;
  static final double     EOPD_THRESHOLD    = .21 ; //reflectance value of floor +refectance value of line/2


  @Override
  public void runOpMode() throws InterruptedException {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
    HardwareDriverControlled        robot   = new HardwareDriverControlled();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    final double     DRIVE_SPEED             = 0.6;
    final double     TURN_SPEED              = 0.5;
    ColorSensor colorSensor;


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
      robot.init(hardwareMap);

      // Send telemetry message to signify robot waiting;
      telemetry.addData("Status", "Resetting Encoders");    //
      telemetry.update();

      robot.leftfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      robot.rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      idle();

      robot.leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      robot.rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      // Send telemetry message to indicate successful Encoder reset
      telemetry.addData("Path0", "Starting at %7d :%7d",
              robot.leftfrontMotor.getCurrentPosition(),
              robot.leftbackMotor.getCurrentPosition(),
              robot.rightbackMotor.getCurrentPosition(),
              robot.rightfrontMotor.getCurrentPosition());

      telemetry.update();
      // hsvValues is an array that will hold the hue, saturation, and value information.
      float hsvValues[] = {0F, 0F, 0F};

      // values is a reference to the hsvValues array.
      final float values[] = hsvValues;

      // get a reference to the RelativeLayout so we can change the background
      // color of the Robot Controller app to match the hue detected by the RGB sensor.
      final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

      // bPrevState and bCurrState represent the previous and current state of the button.
      boolean bPrevState = false;
      boolean bCurrState = false;

      // bLedOn represents the state of the LED.
      boolean bLedOn = true;

      // get a reference to our ColorSensor object.
//        colorSensor = hardwareMap.colorSensor.get("color sensor");

      // Set the LED in the beginning

        telemetry.update();
        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
          // S3: Forward 24 Inches with 4 Sec timeout
//            while (colorSensor.alpha() > colorSensor.red()){
//                encoderDrive(DRIVE_SPEED, 2,-2, 2,-2, 1);
//            }
//            if(colorSensor.red () < colorSensor.blue())
//            {
//                encoderDrive(DRIVE_SPEED, 1,1,1,1,1);
//            }
//
//            else if (colorSensor.blue() < colorSensor.red())
//            {
//                encoderDrive(DRIVE_SPEED, 1,1,1,1,1);
//            }
//            else
//            {
//                encoderDrive(DRIVE_SPEED, -1,-1,-1,-1,1);
//            }

//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
      }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
  public void encoderDrive(double speed,
                           double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                           double timeoutS) throws InterruptedException {
    int newLeftFrontTarget;
    int newRightFrontTarget;
    int newLeftBackTarget;
    int newRightBackTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

//      // Determine new target position, and pass to motor controller
//      newLeftFrontTarget = robot.leftfrontMotor.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
//      newRightFrontTarget = robot.rightfrontMotor.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
//      robot.leftfrontMotor.setTargetPosition(newLeftFrontTarget);
//      robot.rightfrontMotor.setTargetPosition(newRightFrontTarget);
//
//      newLeftBackTarget = robot.leftbackMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
//      newRightBackTarget = robot.rightbackMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
//      robot.leftbackMotor.setTargetPosition(newLeftBackTarget);
//      robot.rightbackMotor.setTargetPosition(newRightBackTarget);
//
//
//      // Turn On RUN_TO_POSITION
//      robot.leftfrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      robot.leftbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      robot.rightfrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      robot.rightbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//      // reset the timeout time and start motion.
//      runtime.reset();
//      robot.leftfrontMotor.setPower(Math.abs(speed));
//      robot.leftbackMotor.setPower(Math.abs(speed));
//      robot.rightfrontMotor.setPower(Math.abs(speed));
//      robot.rightbackMotor.setPower(Math.abs(speed));
//
//      // keep looping while we are still active, and there is time left, and both motors are running.
//      while (opModeIsActive() &&
//              (runtime.seconds() < timeoutS) &&
//              (robot.leftfrontMotor.isBusy() && robot.rightfrontMotor.isBusy() && robot.leftbackMotor.isBusy() && robot.rightbackMotor.isBusy())) {
//
//        // Display it for the driver.
//        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
//        telemetry.addData("Path2",  "Running at %7d :%7d",
//                robot.leftfrontMotor.getCurrentPosition(),
//                robot.rightfrontMotor.getCurrentPosition(),
//                robot.leftbackMotor.getCurrentPosition(),
//                robot.rightbackMotor.getCurrentPosition());
//        telemetry.update();
//
//        // Allow time for other processes to run.
//        idle();
//      }
//
//      // Stop all motion;
//      robot.leftfrontMotor.setPower(0);
//      robot.leftbackMotor.setPower(0);
//      robot.rightfrontMotor.setPower(0);
//      robot.rightbackMotor.setPower(0);
//
//      // Turn off RUN_TO_POSITION
//      robot.leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      robot.leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      robot.rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      robot.rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  sleep(250);   // optional pause after each move
}


  // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // send the info back to driver station using telemetry function.
      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());

      telemetry.update();
      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }
}
