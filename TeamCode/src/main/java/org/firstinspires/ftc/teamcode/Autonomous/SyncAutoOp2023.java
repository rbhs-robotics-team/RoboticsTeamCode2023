/**
*       Non-blocking Hardware Interface
**/

package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.syncop.*;

import java.util.function.Function;

public abstract class SyncAutoOp2023 extends LinearOpMode {
    private BaseController base = null;
    private SlideController slide = null;
    private ClawController claw = null;

    protected ElapsedTime runtime = new ElapsedTime();
    public Telemetry telemetry = null;

    public void initialize(HardwareMap hardware_map, Telemetry telemetry){
        this.telemetry = telemetry;

        Function<Boolean, Boolean> op_mode_is_active_pointer = (Boolean x) -> opModeIsActive();

        base = new BaseController(hardware_map, this, op_mode_is_active_pointer);
        slide = new SlideController(hardware_map, this, op_mode_is_active_pointer);
        claw = new ClawController(hardware_map, this, op_mode_is_active_pointer);
    }

    public void shutdown(){
        base.shutdown();
        slide.shutdown();
        claw.shutdown();
    }

    // packaged functions
    public boolean busy(){
        return base.busy() || slide.busy() || claw.busy();
    }

    public void sync(){
        while(opModeIsActive() && busy()){
            telemetry.addData("Path", "Base{%s} Slide{%s} Claw{%s}", base.busy() ? "T" : "F", slide.busy() ? "T" : "F", claw.busy() ? "T" : "F");
            telemetry.update();
        }
    }

    public void resetZeroHeading() { base.resetZeroHeading(); }

    // component functions
    public void forward(double tiles, double power){ base.forward(tiles, power); }
    public void forward(double tiles){ base.forward(tiles); }
    public void backward(double tiles, double power){ base.backward(tiles, power); }
    public void backward(double tiles){ base.backward(tiles); }
    public void strafe_right(double tiles, double power){ base.strafe_right(tiles, power); }
    public void strafe_right(double tiles){ base.strafe_right(tiles); }
    public void strafe_left(double tiles, double power){ base.strafe_left(tiles, power); }
    public void strafe_left(double tiles){ base.strafe_left(tiles); }

    public void right(double angle){ base.right_gyro(angle); }
    public void left(double angle){ base.left_gyro(angle); }
    
    public void lift(String position, boolean drop){ slide.lift(position, drop); }
    public void lift(String position){ slide.lift(position, false); }
    
    public void grasp(){ claw.grasp(); }
    public void open(){ claw.open(false); }
    public void open(boolean wide){ claw.open(wide); }

    public void pause(double seconds){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            telemetry.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
            telemetry.update();
        }
    }
}