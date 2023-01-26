package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOp2023 extends LinearOpMode {
    protected DcMotor leftfront = null;
    protected DcMotor leftback = null;
    protected DcMotor rightfront = null;
    protected DcMotor rightback = null;
    protected DcMotor lift = null;
    protected DcMotor claw = null;


    protected BNO055IMU imu = null;
    protected double zero_heading = 0.0;
    private Orientation angles = null;

    protected ElapsedTime runtime = new ElapsedTime();
    protected Telemetry telemetry_ = null;

    private double fbMod = 0.49; // Distance modifier for encoder based forward and backward functions
    private double strafeMod = 0.58; // Distance modifier for encoder based strafing functions
    private double quitSpeed = 2.0; // Determines if encoder based wheels are taking too long
    private double speed = 0.77; // Used for @deprecated time based robot movement
    private double strafeSpd = 1.2; // Used for @deprecated time based robot movement
    private double turningCorrection = 0.001; // Used for @deprecated time based robot strafing
    private double turn90 = 0.73; // Used for @deprecated time based robot rotation
    protected double wheelPower = 0.8; // Wheel speed
    private double margin = 1; // Margin of error for gyrometer based robot rotation
    private double ticksPerTile = 1120/4/Math.PI*24*1.0; // Used for encoder based wheel movement (1120 ticks/rev * 1/4pi rev/in * 24 in/tile * gear_ratio)


    /** Initialization **/
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        leftfront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftback = hardwareMap.get(DcMotor.class, "backLeft");
        rightfront = hardwareMap.get(DcMotor.class, "frontRight");
        rightback = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(DcMotor.class, "claw");
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.FORWARD);
        telemetry_ = telemetry;
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
        // Set zero power behavior to brake.
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, OpenCvCamera camera){
        initialize(hardwareMap, telemetry);
    }



    /** Low-Level Conviniences **/
    public void leftWheels(double spd){
        leftfront.setPower(spd);
        leftback.setPower(spd);
    }
    public void rightWheels(double spd){
        rightfront.setPower(spd);
        rightback.setPower(spd);
    }


    /** Speed-Based Movement **/
    public void driveSpd(double spd){
        leftWheels(spd);
        rightWheels(spd);
    }
    public void turnRightSpd(double spd){
        leftWheels(spd);
        rightWheels(-spd);
    }
    public void turnLeftSpd(double spd){
        leftWheels(-spd);
        rightWheels(spd);
    }
    public void strafeLeftSpd(double spd, double corr){
        leftfront.setPower(-spd+corr);
        leftback.setPower(spd+corr);
        rightfront.setPower(spd-corr);
        rightback.setPower(-spd-corr);
    }
    public void strafeLeftSpd(double spd){
        strafeLeftSpd(spd, 0.0);
    }
    public void strafeRightSpd(double spd, double corr){
        leftfront.setPower(spd+corr);
        leftback.setPower(-spd+corr);
        rightfront.setPower(-spd-corr);
        rightback.setPower(spd-corr);
    }
    public void strafeRightSpd(double spd){
        strafeRightSpd(spd, 0.0);
    }


    /** Encoder-Based Movement **/
    public void move(double y_tiles, double x_tiles, String name){
        resetWheels();
        leftfront.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        leftback.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightfront.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightback.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        setWheelMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setPower(wheelPower);
        leftback.setPower(wheelPower);
        rightfront.setPower(wheelPower);
        rightback.setPower(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (leftfront.isBusy() || rightfront.isBusy() || leftback.isBusy() || rightback.isBusy()) && runtime.seconds() < Math.sqrt(y_tiles*y_tiles+x_tiles*x_tiles)*quitSpeed/wheelPower){
            telemetry_.addData("Path", "%s: %s lf, %s rf, %s lb, %s rb", name, leftfront.getCurrentPosition(), rightfront.getCurrentPosition(), leftback.getCurrentPosition(), rightback.getCurrentPosition());
            telemetry_.update();
        }
        leftfront.setPower(0.0);
        leftback.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double y_tiles, double x_tiles){
        // negate y_tiles to account for motors being in reverse mode
        move(-y_tiles, x_tiles, "Moving");
    }

    public void strafeLeft_enc(double tiles){
        move(0.0, -tiles, "Strafing Right");
    }
    public void strafeRight_enc(double tiles){
        move(0.0, tiles, "Strafing Left");
    }
    public void forward_enc(double tiles){
        move(tiles, 0.0, "Forward");
    }
    public void backward_enc(double tiles){
        move(-tiles, 0.0, "Backward");
    }


    /** Encoder Utilities **/
    public void setWheelMode(DcMotor.RunMode mode){
        leftfront.setMode(mode);
        leftback.setMode(mode);
        rightfront.setMode(mode);
        rightback.setMode(mode);
    }
    public void resetWheels(){
        setWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getVector(){
        double[] offset = new double[2];
        double lf = leftfront.getCurrentPosition() / ticksPerTile;
        double lb = leftback.getCurrentPosition() / ticksPerTile;
        double rf = rightfront.getCurrentPosition() / ticksPerTile;
        double rb = rightback.getCurrentPosition() / ticksPerTile;
        offset[0] = (lf + lb + rf + rb) / 4;
        offset[1] = (lf - lb - rf + rb) / 4;
        return offset;
    }


    /** Gyrometer Access **/
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360 + margin) % 360.0;
    }
    public double getNegAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading - 720 - margin) % 360.0;
    }
    public double getSmAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360) % 360.0 - 180.0;
    }
    @Deprecated
    public void zeroHeading(){
        //double off = getAngle();
        //zero_heading += off;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
    }


    /** Gyroscope-Based Rotation **/
    public void left_gyro(double quarters){
        if(quarters > 2){
            left_gyro(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        pause(0.05);
        while(opModeIsActive() && ((a=getAngle()) < 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(wheelPower*0.2);
        rightWheels(-wheelPower*0.2);
        while(opModeIsActive() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading + 90.0) % 360.0;
    }
    public void right_gyro(double quarters){
        if(quarters > 2){
            right_gyro(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getNegAngle()%360) < -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        pause(0.05);
        while(opModeIsActive() && ((a=getNegAngle()) > -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(-wheelPower*0.2);
        rightWheels(wheelPower*0.2);
        while(opModeIsActive() && ((a=getNegAngle()) < -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading - 90.0) % 360.0;
    }
    public void turnZero(double margin, String name){
        double a = getSmAngle();
        runtime.reset();
        while(Math.abs(a) > margin){
            leftWheels(a*0.005);
            rightWheels(-a*0.005);
            telemetry_.addData("Path", "%s: %2.5f S Elapsed, %2.3", name, a, runtime.seconds());
            telemetry_.update();
            a = getSmAngle();
        }
        stopWheels();
    }
    public void turnZero(double margin){
        turnZero(margin, "Turning");
    }
    public void turnZero(){
        turnZero(5.0);
    }


    /** Time-Based Movement (Backwards-Compatibility) **/
    @Deprecated
    public void strafeLeft_time(double tiles){
        zeroHeading();
        strafeLeftSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafeSpd / wheelPower)){
            telemetry_.addData("Path", "Strafing Left: %2.5f S Elapsed", runtime.seconds());
            strafeLeftSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }
    @Deprecated
    public void strafeRight_time(double tiles){
        zeroHeading();
        strafeRightSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafeSpd / wheelPower)){
            telemetry_.addData("Path", "Strafing Right: %2.5f S Elapsed", runtime.seconds());
            strafeRightSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }
    @Deprecated
    public void forward_time(double tiles){
        leftWheels(wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void backward_time(double tiles){
        leftWheels(-wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Backward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void left_time(double quarters){
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void right_time(double quarters){
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), getAngle());
            telemetry_.update();
        }
        stopWheels();
    }


    /** Movement Interface **/
    public void strafeRight(double tiles){
        strafeRight_enc(tiles*strafeMod);
    }
    public void strafeRight(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        strafeRight_enc(tiles*strafeMod);
        wheelPower = temp;
    }
    public void strafeLeft(double tiles){
        strafeLeft_enc(tiles*strafeMod);
    }
    public void strafeLeft(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        strafeLeft_enc(tiles*strafeMod);
        wheelPower = temp;
    }
    public void forward(double tiles){
       forward_enc(tiles*fbMod);
    }
    public void forward(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        forward_enc(tiles*fbMod);
        wheelPower = temp;
    }
    public void backward(double tiles){
        backward_enc(tiles*fbMod);
    }
    public void backward(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        backward_enc(tiles*fbMod);
        wheelPower = temp;
    }
    public void left(double quarters){
        left_gyro(quarters);
    }
    public void right(double quarters){
        right_gyro(quarters);
    }


    /** Mechanisms **/
    /*
    @Deprecated
    public void armUp(){
        arm.setPower(0.5);
    }
    @Deprecated
    public void armDown(){
        arm.setPower(-0.5);
    }

    @Deprecated
    public void armUp(double pct){
        arm.setPower(armStatic+armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct)){
            telemetry_.addData("Path", "Raising arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(armStatic);
    }
    @Deprecated
    public void armDown(double pct){
        arm.setPower(armStatic-armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct)){
        }
        arm.setPower(armStatic);
    }

    @Deprecated
    public void armUpEnc(int pos){
        runtime.reset();
        while(opModeIsActive() && (arm.getCurrentPosition() < pos) && (runtime.seconds() < Math.abs(pos-arm.getCurrentPosition())/100)){
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setPower(armPower);
        }
        arm.setPower(armStatic);
    }
    @Deprecated
    public void armDownEnc(int pos){
        runtime.reset();
        while(opModeIsActive() && (arm.getCurrentPosition() > pos) && (runtime.seconds() < Math.abs(pos-arm.getCurrentPosition())/100)){
            arm.setDirection(DcMotor.Direction.FORWARD);
            arm.setPower(armPower);
        }
        arm.setPower(armStatic);
    }
    */

    // Encoder movement for arm
    public void lift(String posString){
        int pos = lift.getCurrentPosition();
        if(posString == "high"){
            pos = 3000;
        }
        else if(posString == "highDrop"){
            pos = 2600;
        }
        else if(posString == "medium"){
            pos = 2150;
        }
        else if(posString == "mediumDrop"){
            pos = 1900;
        }
        else if(posString == "low"){
            pos = 1300;
        }
        else if(posString == "lowDrop"){
            pos = 980;
        }
        else if(posString == "max"){
            pos = 3200;
        }
        else if(posString == "min" || posString == "ground"){
            pos = 0;
        }
        lift.setTargetPosition(pos);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.5);
        while(opModeIsActive() && lift.isBusy()) {
            telemetry_.addData("Lift", lift.getCurrentPosition());
            telemetry_.update();
        }
        //lift.setPower(armStatic);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    // Position based claw movement
    public void openClaw(){
        claw.setPosition(1);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Opening Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void closeClaw(){
        claw.setPosition(0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Closing Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
  */


    /** Miscelaneous Utilities **/
    public void pause(double seconds){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            telemetry_.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
            telemetry_.update();
        }
    }
    public void pause(double seconds, boolean telem){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            if(telem) {
                telemetry_.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
                telemetry_.update();
            }
        }
    }

    public void stopWheels(){
        leftWheels(0.0);
        rightWheels(0.0);
    }
}