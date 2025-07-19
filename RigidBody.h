#pragma once
#include <wx/wx.h>

class RigidBody {
    public:
        RigidBody();
        RigidBody(wxRealPoint startPosition, wxRealPoint startLinearVelocity, double startAngle, double startAngularVelocity, double diagonalLength, double mass, double moment);

        void Update(double dt);
        void ApplyImpulse(wxRealPoint impulse, wxRealPoint contactPoint);
        void SetDragged(bool dragging, wxRealPoint pivot);
        void SetPosition(wxRealPoint newPosition);
        void SetVelocity(wxRealPoint newVelocity);
        void SetMouseVelocity(wxRealPoint newVelocity);
        void PushMousePosition(wxRealPoint newPosition);

        wxRealPoint GetPosition();
        double GetAngle();
        wxRect2DDouble ComputeAABB();
        void PerformCollision();
    private:
        wxRealPoint centerPosition; // pixels
        wxRealPoint linearVelocity; // meters per second
        double rotationAngle; // degrees
        double angularVelocity; // degrees per second
        double mass; // kg 
        double momentOfInertia; // kg * meter squared
        double diagonalLength; // pixels
        bool beingDragged;
        wxRealPoint dragPivot; // pixels
        wxRealPoint mouseVelocity; // meters per second
        wxRealPoint currentMousePosition; // pixels
};