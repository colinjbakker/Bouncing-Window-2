#pragma once
#include <wx/wx.h>
#include "RigidBody.h"
#include <queue>

class Frame : public wxFrame
{
public:
    Frame(wxPoint startPosition, wxRealPoint startLinearVelocity, double startAngle, double startAngularVelocity, wxSize windowSize);
    
private:
    wxTimer* tickTimer = nullptr;
    wxPanel* clickablePanel = nullptr;
    wxPanel* controlPanel = nullptr;
    wxPanel* closePanel = nullptr;
    wxPanel* minimizePanel = nullptr;
    RigidBody rigidBody;

    wxRealPoint position; // pixels
    wxRealPoint previousMousePosition; //pixels
    wxRealPoint currentMousePosition; // pixels

    double timeAccumulator = 0; // seconds
    std::__1::chrono::steady_clock::time_point previous; // time point

    wxPoint dragStartScreen; // pixels
    wxPoint dragOffset; // pixels

    bool dragging = false;
    bool closeButtonHovering = false;
    bool windowIsFocused = true;
    double rotationAngle; // degrees
    int red = 0x3D;
    int green = 0x42;
    int blue = 0x4A;
    
    
    int displayHeight = wxGetDisplaySize().GetHeight(); // pixels
	int displayWidth = wxGetDisplaySize().GetWidth(); // pixels

    wxSize windowSize; // pixels x pixels

    wxGraphicsContext *gc;

    void OnTick(wxTimerEvent& event);
    void OnActivate(wxActivateEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    void OnMouseUp(wxMouseEvent& event);
    void OnMouseDown(wxMouseEvent& event);
    void OnPaint(wxPaintEvent& event);
    void OnPanelPaint(wxPaintEvent& event);
    void OnClosePanelPaint(wxPaintEvent& event);
    void OnExitClick(wxMouseEvent& event);
    void OnMouseEnterClosePanel(wxMouseEvent& event);
    void OnMouseLeaveClosePanel(wxMouseEvent& event);
    void UpdateClosePanel();
    void HandleTick(double dtSeconds);
    void UpdatePosition();
    wxDECLARE_EVENT_TABLE();
};