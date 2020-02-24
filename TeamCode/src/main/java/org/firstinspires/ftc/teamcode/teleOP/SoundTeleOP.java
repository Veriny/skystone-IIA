package org.firstinspires.ftc.teamcode.teleOP;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SoundSystem;


@TeleOp(name="SoundTeleOp", group = "DrivetrainTests")

public class SoundTeleOP extends OpMode{
    public SoundSystem st;
    public MediaPlayer player;
    @Override
    public void init() {
        st = new SoundSystem(player);
    }

    @Override
    public void start() {
        super.start();
        telemetry.addLine("Let's make some music boys");
    }


    @Override
    public void loop() {
        st.c(gamepad1, hardwareMap, telemetry);
    }
}