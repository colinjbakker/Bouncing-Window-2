#pragma once
#include <wx/wx.h>
#include <random>

class Helper {
    public:
        static unsigned constexpr int INTERVAL = 4; // milliseconds
        static constexpr double TIMESTEP = 0.008333; // seconds, 120 fps
        static constexpr double PIXELS_PER_METER = 100.0;

        static constexpr double GRAVITY = 9.81;

        static constexpr double AIR_RESISTANCE = 0.05;
        static constexpr double AIR_RESISTANCE_ROTATION = 0.3;

        static constexpr double RESTITUTION = 0.3;
        static constexpr double FRICTION = 0.4;


        static wxColor getNextColor(int* red, int* green, int* blue);
        static double degreesToRadians(double degrees);
        static std::vector<wxRealPoint> GetWorldCorners(double rotationAngle, double diagonalLength, wxRealPoint centerPosition);
        static std::vector<wxRealPoint> GetObjectCorners(double rotationAngle, double diagonalLength);
        static wxGraphicsPath DrawPath(double rotationAngle, int cornerRadius, double diagonalLength, wxGraphicsContext* gc);
        static wxRealPoint rotatePointByDegrees(wxRealPoint point, double degrees);
        static wxRealPoint rotatePointByRadians(wxRealPoint point, double radians);
        static double crossProduct(wxRealPoint p1, wxRealPoint p2);
        static int sign(double val);

        static wxRealPoint toMeters(wxRealPoint pointPixels);
        static wxRealPoint toPixels(wxRealPoint pointMeters);


};