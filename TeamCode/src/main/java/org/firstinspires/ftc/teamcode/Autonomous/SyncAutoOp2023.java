/**
*       Non-blocking Hardware Interface
**/

package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.syncop.BaseController;

public abstract class SyncAutoOp2023 extends LinearOpMode {
    private BaseController base = null;
    private SlideController slide = null;
    private ClawController claw = null;

    private Telemetry telemetry = null;

    public void initialize(HardwareMap hardware_map, Telemetry telemetry){
        this.telemetry = telemetry;

        Function<boolean, boolean> op_mode_is_active_pointer = (boolean x) -> { return opModeIsActive(); }
        
        base = new BaseController(hardware_map, telemetry, op_mode_is_active_pointer);
        slide = new SlideController(hardware_map, telemetry, op_mode_is_active_pointer);
        claw = new ClawController(hardware_map, telemetry, op_mode_is_active_pointer);
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

    // component functions
    public void forward(double tiles, double power){ base.forward(tiles, power); }

    public void forward(double tiles){ base.forward(tiles); }

    public void backward(double tiles, double power){ base.backward(tiles, power); }

    public void backward(double tiles){ base.backward(tiles); }

    public void strafe_right(double tiles, double power){ base.strafe_right(tiles, power); }

    public void strafe_right(double tiles){ base.strafe_right(tiles); }

    public void strafe_left(double tiles, double power){ base.strafe_left(tiles, power); }

    public void strafe_left(double tiles){ base.strafe_left(tiles); }

    public void lift(String position){ slide.lift(position); }
}