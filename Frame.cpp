#include "Frame.h"
#include "Helper.h"

wxBEGIN_EVENT_TABLE(Frame, wxFrame)
    EVT_TIMER(10001, Frame::OnTick)
wxEND_EVENT_TABLE()

Frame::Frame(wxPoint startPosition, wxRealPoint startLinearVelocity, double startAngle, double startAngularVelocity, wxSize windowSize) : wxFrame(nullptr, wxID_ANY, "Bouncing Window", startPosition, windowSize, wxFRAME_SHAPED | wxSTAY_ON_TOP), tickTimer(new wxTimer(this, 10001)), clickablePanel(new wxPanel(this, 10002, wxPoint(0,0), windowSize)), position(startPosition), rotationAngle(startAngle), windowSize(windowSize), previous(std::chrono::steady_clock::now())
{
    currentMousePosition = wxGetMousePosition();
    tickTimer->Start(Helper::INTERVAL);
    
    clickablePanel->SetFocus();
    clickablePanel->Bind(wxEVT_LEFT_DOWN, &Frame::OnMouseDown, this);
    clickablePanel->Bind(wxEVT_CHAR_HOOK, &Frame::OnKeyDown, this);
    clickablePanel->Bind(wxEVT_PAINT, &Frame::OnPanelPaint, this);

    rigidBody = RigidBody(position, startLinearVelocity, rotationAngle, startAngularVelocity, windowSize.GetWidth(), 1.0, 0.05);

    // controlPanel = new wxPanel(clickablePanel, wxID_ANY, wxPoint(0,0), wxSize()

    closePanel = new wxPanel(clickablePanel, wxID_ANY, wxPoint(0,0), wxSize(16,16));
    closePanel->SetBackgroundStyle(wxBG_STYLE_PAINT);
    closePanel->Bind(wxEVT_PAINT, &Frame::OnClosePanelPaint, this);
    closePanel->Bind(wxEVT_LEFT_DOWN, &Frame::OnExitClick, this);
    closePanel->Bind(wxEVT_ENTER_WINDOW, &Frame::OnMouseEnterClosePanel, this);
    closePanel->Bind(wxEVT_LEAVE_WINDOW, &Frame::OnMouseLeaveClosePanel, this);

    Bind(wxEVT_LEFT_UP, &Frame::OnMouseUp, this);
    Bind(wxEVT_PAINT, &Frame::OnPaint, this);
    Bind(wxEVT_ACTIVATE, &Frame::OnActivate, this);
}

void Frame::OnKeyDown(wxKeyEvent& event){
    if (event.GetKeyCode() == WXK_ESCAPE) {
        tickTimer->Stop();
        Close(true);
    }
    event.Skip();
}

void Frame::OnExitClick(wxMouseEvent& event){
    tickTimer->Stop();
    Close(true);
}
void Frame::OnActivate(wxActivateEvent& event){
    bool isActive = event.GetActive();

    windowIsFocused = isActive;
    closePanel->Refresh();

    event.Skip();
}

void Frame::OnTick(wxTimerEvent& event){
    std::__1::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double deltaTime = std::chrono::duration<double>(now - previous).count();
    previous = now;
    timeAccumulator += deltaTime;
    previousMousePosition = currentMousePosition;
    currentMousePosition = wxGetMousePosition();
    rigidBody.PushMousePosition(currentMousePosition);
    rigidBody.SetMouseVelocity(wxRealPoint((currentMousePosition.x - previousMousePosition.x) / deltaTime, (currentMousePosition.y - previousMousePosition.y) / deltaTime));

    while (timeAccumulator >= Helper::TIMESTEP) {
        HandleTick(Helper::TIMESTEP);
        timeAccumulator -= Helper::TIMESTEP;
    }
}

void Frame::OnMouseDown(wxMouseEvent& event) {
    dragging = true;
    CaptureMouse();
    previous = std::chrono::steady_clock::now();

    wxRealPoint relativePivot = wxGetMousePosition() - (GetPosition() + wxPoint(windowSize.GetWidth()/2, windowSize.GetHeight()/2)); //mouse position world - center of box world
    wxRealPoint relativePivotRotated = Helper::rotatePointByDegrees(relativePivot, rotationAngle);

    rigidBody.SetDragged(true, relativePivotRotated);
    event.Skip();
}

void Frame::OnMouseUp(wxMouseEvent& event) {
    dragging = false;
    wxRealPoint relativePivot = wxGetMousePosition() - (GetPosition() + wxPoint(windowSize.GetWidth()/2, windowSize.GetHeight()/2)); //mouse position world - center of box world
    wxRealPoint relativePivotRotated = Helper::rotatePointByDegrees(relativePivot, rotationAngle);
    rigidBody.SetDragged(false, relativePivotRotated);
    if (HasCapture()) ReleaseMouse();
    event.Skip();
}

void Frame::OnPaint(wxPaintEvent& event) {
    wxPaintDC dc(this);
    auto gc = wxGraphicsContext::Create(dc);
    if (!gc) return;

    wxGraphicsPath p = Helper::DrawPath(rotationAngle, 9, windowSize.GetX() / 2, gc);
    SetShape(p);
    delete gc;
}

void Frame::OnPanelPaint(wxPaintEvent& event) {
    wxPaintDC dc(clickablePanel);
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (!gc) return;

    gc->SetAntialiasMode(wxANTIALIAS_DEFAULT);
    gc->SetFont(wxFontInfo(12).Bold(), wxColour(0xd4d4d4));

    wxString text = "Bouncing Window";

    wxSize size = clickablePanel->GetSize();
    wxDouble centerX = size.GetWidth() / 2.0;
    wxDouble centerY = size.GetHeight() / 2.0;

    wxDouble textWidth, textHeight;
    gc->GetTextExtent(text, &textWidth, &textHeight);

    gc->PushState();
    gc->Translate(centerX, centerY);
    gc->Rotate(Helper::degreesToRadians(-rotationAngle+45));
    gc->DrawText(text, -textWidth / 2.0, -textHeight / 2.0);
    gc->PopState();
}

void Frame::OnClosePanelPaint(wxPaintEvent& event) {
    wxPaintDC dc(closePanel);
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (!gc) return;

    wxSize size = closePanel->GetSize();
    
    double cx = size.GetWidth() / 2.0;
    double cy = size.GetHeight() / 2.0;
    double r = std::min(cx, cy) - 1;
    
    if(windowIsFocused || closeButtonHovering){
        gc->SetBrush(gc->CreateBrush(wxBrush(wxColour("#FF605C"))));
        gc->DrawEllipse(cx - r, cy - r, 2 * r, 2 * r);
    } else {
        gc->SetBrush(gc->CreateBrush(wxBrush(wxColour("#616161ff"))));
        gc->DrawEllipse(cx - r, cy - r, 2 * r, 2 * r);
    }
    

    if (closeButtonHovering) {
        double lineLen = r * 0.4;
        wxRealPoint line1_start = Helper::rotatePointByDegrees({-lineLen, -lineLen}, -rotationAngle -45);
        wxRealPoint line1_end   = Helper::rotatePointByDegrees({lineLen, lineLen}, -rotationAngle - 45);

        wxRealPoint line2_start = Helper::rotatePointByDegrees({-lineLen, lineLen}, -rotationAngle -45);
        wxRealPoint line2_end   = Helper::rotatePointByDegrees({lineLen, -lineLen}, -rotationAngle -45);

        gc->SetPen(wxPen(wxColour("#750000ff"), 2));

        gc->StrokeLine(cx + line1_start.x, cy + line1_start.y, cx + line1_end.x, cy + line1_end.y);
        gc->StrokeLine(cx + line2_start.x, cy + line2_start.y, cx + line2_end.x, cy + line2_end.y);
    }
}

void Frame::OnMouseEnterClosePanel(wxMouseEvent& event){
    closeButtonHovering = true;
    event.Skip();
}
void Frame::OnMouseLeaveClosePanel(wxMouseEvent& event){
    closeButtonHovering = false;
    event.Skip();
}

void Frame::HandleTick(double dtSeconds){
    rigidBody.Update(dtSeconds);
    position = rigidBody.GetPosition() - wxRealPoint(windowSize.GetWidth() / 2, windowSize.GetHeight() / 2);
    rotationAngle = rigidBody.GetAngle();
    UpdatePosition();
    UpdateClosePanel();
    Refresh();
    clickablePanel->Refresh(false);
    closePanel->Refresh(false);
}

void Frame::UpdateClosePanel(){
    wxRealPoint offset(0.0, -(windowSize.GetY() / 2.0 - 20.0));
    wxRealPoint center(windowSize.x / 2.0, windowSize.y / 2.0);
    wxRealPoint btnPos = center + Helper::rotatePointByDegrees(offset, -rotationAngle);
    wxSize btnSize = closePanel->GetSize();
    closePanel->Move(btnPos - wxRealPoint(btnSize.GetWidth() / 2, btnSize.GetHeight() / 2));
}

void Frame::UpdatePosition(){
    this->Move(wxPoint(std::round(position.x), std::round(position.y)));
}
