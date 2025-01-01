#pragma once

namespace Constants {
    constexpr char CameraTopic[]    = "/camera/image_raw";
    constexpr char TwistTopic[]     = "/cmd_vel";
    constexpr int  QueueSize        = 10;
    constexpr double ThresholdValue  = 20.0;
    constexpr double LinearSpeedMax  = 0.3;
    constexpr int  CircleRadius      = 5;
    constexpr int  MiddleCircleRadius = 3;
    constexpr double AngularSpeedScale = 0.0008;
    constexpr double MinExtent        = 0.2;
    constexpr int  ControlLoopPeriodMs = 10;  // milisaniye
    constexpr int  StopDurationMs   = 1800;   // milisaniye
    constexpr double TurnLeftDurationSec = 4.05;  // saniye
    constexpr double TurnRightDurationSec = 4.05;  // saniye
    constexpr double TurnSpeed  = 0.3;
    constexpr int  BlackPixelValue = 300;
    constexpr int  LeftYStartValue = 420;
    constexpr int  LeftYEndValue = 480;
    constexpr int  LeftXStartValue = 0;
    constexpr int  LeftXEndValue = 5;
    constexpr int  RightYStartValue = 420;
    constexpr int  RightYEndValue = 480;
    constexpr int  RightXStartValue = 635;
    constexpr int  RightXEndValue = 640;
    constexpr double MinimumArea = 210000.0;
    constexpr int  MidPixelValue = 255;
}

