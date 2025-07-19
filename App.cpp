#include "App.h"
#include "Frame.h"

bool App::OnInit()
{
    Frame *frame = new Frame(wxPoint(500, 500), wxRealPoint(5, -5), 25.0, 250.0, wxSize(200, 200));
    frame->Show(true);
    return true;
}