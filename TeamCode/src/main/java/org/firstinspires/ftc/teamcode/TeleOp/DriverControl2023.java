/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="DriverControl2023", group="Linear OpMode")

public class DriverControl2023 extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor lift = null;
    private DcMotor claw = null;

    // Gyro testing
    protected BNO055IMU imu = null;
    private double zero_heading = 0.0;
    private double zero_y = 0.0;
    private Orientation angles = null;

    // Gyro access
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading+360) % 360.0;
    }
    public void zeroHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading  = angles.firstAngle;
        zero_y = angles.secondAngle;
    }

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive_0  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "frontRight");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "backLeft");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(DcMotor.class, "claw");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.FORWARD);

        // Reset Encoders
        leftDrive_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set initial motor target positions
        lift.setTargetPosition(0);
        lift.setPower(1);
        claw.setTargetPosition(0);
        claw.setPower(.1);

        // Set motors with encoders to run at constant speed
        leftDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set zero power behavior to brake
        leftDrive_0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive_0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Variables
        double leftPower, rightPower, strafe, vbrake; // wheel speed modifiers
        double mod = .6; // overall wheel speed modifier
        double targetLF, targetLB, targetRF, targetRB; // target wheel speeds
        double actLF=0.0, actLB=0.0, actRF=0.0, actRB=0.0;  // current wheel speeds
        double delta = 0.15; // acceleration (power change per iteration -> lower number=larger acceleration)
        int liftPosition = 0; //current position of lift
        int clawPosition = 0; // current position of claw

        boolean clawAlt = true;
        boolean liftAlt = false;
        double altDelayLength = 2;
        ElapsedTime altDelayLift = new ElapsedTime();
        ElapsedTime altDelayClaw = new ElapsedTime();

        // Gyro Config
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        // params.accelerationIntegerationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
        zeroHeading();

        telemetry.addData("Status", "Beans has been initialized.");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get movement commands
            if(gamepad1.dpad_up) {
                leftPower = -1;
                rightPower = -1;
            }
            else if(gamepad1.dpad_down) {
                leftPower = 1;
                rightPower = 1;
            }
            else {
                leftPower  = (Math.abs(gamepad1.left_stick_y)*100)*((gamepad1.left_stick_y)*100)/10000;
                rightPower = gamepad1.right_stick_y;
            }


            if(gamepad1.dpad_right) {
                strafe = .5;
            }
            else if(gamepad1.dpad_left) {
                strafe = -.5;
            }
            else if(gamepad1.right_trigger>0) {
                strafe = .5*(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger>0) {
                strafe = -.5*(gamepad1.left_trigger);
            }
            else {
                strafe = 0;
            }

            // Get braking and slow mode
            if(gamepad1.left_bumper) {
                vbrake = 0;
            }
            else if(gamepad1.right_bumper) {
                vbrake = .25;
            } else {
                vbrake = 1;
            }
            leftPower *= vbrake;
            rightPower *= vbrake;
            strafe*=vbrake;
            telemetry.addData("Braking", "Braking at: (%2.0f) %%", (100-vbrake*100));

            // Apply overall wheel speed modifier
            leftPower *= mod;
            rightPower *= mod;
            strafe *= mod;


            // Calculate target and actual wheel speeds
            targetLF = leftPower - strafe;
            if(actLF < targetLF - delta){
                actLF += delta;
            }
            else if(actLF > targetLF + delta){
                actLF -= delta;
            }
            else{
                actLF = targetLF;
            }

            targetLB = leftPower + strafe;
            if(actLB < targetLB - delta){
                actLB += delta;
            }
            else if(actLB > targetLB + delta){
                actLB -= delta;
            }
            else{
                actLB = targetLB;
            }

            targetRF = rightPower + strafe;
            if(actRF < targetRF - delta){
                actRF += delta;
            }
            else if(actRF > targetRF + delta){
                actRF -= delta;
            }
            else{
                actRF = targetRF;
            }

            targetRB = rightPower - strafe;
            if(actRB < targetRB - delta){
                actRB += delta;
            }
            else if(actRB > targetRB + delta){
                actRB -= delta;
            }
            else{
                actRB = targetRB;
            }

            // Send calculated power to wheels
            leftDrive_0.setPower(actLF);
            rightDrive_0.setPower(actRF);
            leftDrive_1.setPower(actLB);
            rightDrive_1.setPower(actRB);

            // Show the elapsed game time and wheel power
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Wheels", "leftFront=(%.2f), leftBack=(%.2f), rightFront=(%.2f), rightBack=(%.2f)", actLF, actLB, actRF, actRB);
            telemetry.addData("Heading", "Z %2.3f, Y %2.3f", getAngle(), (angles.secondAngle-zero_y+360)%360.0);

            // Lift alternate mode
            if(gamepad2.y & altDelayLift.seconds()>altDelayLength) {
                liftAlt = !liftAlt;
                altDelayLift.reset();
            }
            if(liftAlt) {

            }
            else {
                if (gamepad2.right_trigger > 0) {
                    lift.setTargetPosition(3200);
                    lift.setPower(gamepad2.right_trigger);
                    liftPosition = lift.getCurrentPosition() + (int) (gamepad2.right_trigger * 30);
                } else if (gamepad2.left_trigger > 0) {
                    lift.setTargetPosition(0);
                    lift.setPower(gamepad2.left_trigger);
                    liftPosition = lift.getCurrentPosition() - (int) (gamepad2.right_trigger * 50);
                } else if (gamepad2.left_stick_y < 0) {
                    lift.setTargetPosition(3200);
                    lift.setPower(Math.abs(gamepad2.left_stick_y));
                    liftPosition = lift.getCurrentPosition() + (int) (-gamepad2.left_stick_y * 30);
                } else if (gamepad2.left_stick_y > 0) {
                    lift.setTargetPosition(0);
                    lift.setPower(Math.abs(gamepad2.left_stick_y));
                    liftPosition = lift.getCurrentPosition() - (int) (-gamepad2.left_stick_y * 50);
                } else {
                    lift.setTargetPosition(liftPosition);
                    lift.setPower(1);
                }
            }

            telemetry.addData("Lift", "position=(%3.0f)", (double) lift.getCurrentPosition());

            // Claw alternate mode
            if(gamepad2.x & altDelayClaw.seconds()>altDelayLength) {
                clawAlt = !clawAlt;
                altDelayClaw.reset();
            }
            // Claw
            if(clawAlt) {
                if (gamepad2.right_bumper) {
                    claw.setPower(100);
                    claw.setTargetPosition(0);
                } else if (gamepad2.left_bumper) {
                    claw.setPower(100);
                    claw.setTargetPosition(-80);
                }
                claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad2.right_bumper) {
                    claw.setPower(.35);
                } else if (gamepad2.left_bumper) {
                    claw.setPower(-.35);
                } else {
                    claw.setPower(0);
                }
            }
            telemetry.addData("Claw", "position=(%3.0f)", (double) claw.getCurrentPosition());



            /*
            if(gamepad2.right_bumper) {
                claw.setTargetPosition(0);
            }
            else if(gamepad2.left_bumper) {
                claw.setTargetPosition(3200);
            }
            else {
                claw.setTargetPosition(claw.getCurrentPosition());
            }

            telemetry.addData("Claw", "position=(%3.0f)", (double) claw.getCurrentPosition());
            */

            /* Servo-based claw
            if(gamepad2.right_bumper) {
                clawPosition -= clawSpeed;
            }
            else if(gamepad2.left_bumper) {
                clawPosition += clawSpeed;
            }

            // minimum position
            if(clawPosition < 0.2) {
                clawPosition = 0.2;
            }
            // maximum position
            if(clawPosition > 0.4) {
                clawPosition = 0.4;
            }

            claw.setPosition(clawPosition);
            telemetry.addData("Claw", "position=(%.2f)", claw.getPosition());
            */
            telemetry.update();
        }
    }

}