#pragma once

namespace Constants {
    constexpr char CameraTopic[]    = "/camera/image_raw";
    constexpr char TwistTopic[]     = "/cmd_vel";
    constexpr int  QueueSize        = 10;
   
    constexpr int  ControlLoopPeriodMs = 10;  // milisaniye

    constexpr double BINARY_THRESHOLD_VALUE  = 20.0;
    constexpr double MinExtent        = 0.2;

    constexpr int  CircleRadius      = 5;
    constexpr int  MiddleCircleRadius = 3;
    
    constexpr int  AdjustmentTimerMs   = 1800;   // milisaniye
    constexpr double TurnLeftDurationSec = 4.05;  // saniye
    constexpr double TurnRightDurationSec = 4.05;  // saniye
    
    constexpr double TurningSpeed  = 0.35;
    constexpr double LinearSpeed  = 0.3;
    constexpr double AngularSpeedScale = 0.0015;

    constexpr int  BlackPixelValue = 300;
    constexpr int  LEFT_Y_START = 420;
    constexpr int  LEFT_Y_END = 480;
    constexpr int  LEFT_X_START = 0;
    constexpr int  LEFT_X_END = 5;
    constexpr int  RIGHT_Y_START = 420;
    constexpr int  RIGHT_Y_END = 480;
    constexpr int  RIGHT_X_START = 635;
    constexpr int  RIGHT_X_END = 640;
    constexpr double MinimumArea = 210000.0;
    constexpr int  MidPixelValue = 255;
}

