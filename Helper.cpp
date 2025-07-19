#include "Helper.h"

wxColor Helper::getNextColor(int* red, int* green, int* blue){
    if(*red >= 0xFF){
        (*red) = 0x00;
    } else {
        (*red)++;
    }

    if(*blue <= 0x00){
        (*blue) = 0xFF;
    } else {
        (*blue)--;
    }

    if(*green >= 0xFF){
        (*green) = 0x00;
    } else {
        (*green)++;
    }

    return wxColor(*red, *green, *blue, 0xFF);
}

double Helper::degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0); 
}

wxGraphicsPath Helper::DrawPath(double rotationAngle, int cornerRadius, double diagonalLength, wxGraphicsContext* gc){
    wxGraphicsPath path = gc->CreatePath();

    wxRealPoint corners[4];
    wxRealPoint lineEndpoints[8];
    int i;
    for(i = 0; i < 4; i++){
        double currAngle = Helper::degreesToRadians(rotationAngle + (90 * i));
        corners[i] = wxRealPoint(diagonalLength * cos(currAngle), diagonalLength * sin(currAngle));
    }

    for(i = 0; i < 4; i++){
        wxRealPoint startPoint = corners[i];
        wxRealPoint endPoint = (i == 3) ? corners[0] : corners[i+1];
        double distance = sqrt((endPoint.x - startPoint.x) * (endPoint.x - startPoint.x) + (endPoint.y - startPoint.y) * (endPoint.y - startPoint.y));
        double t1 = cornerRadius / distance;
        double t2 = 1-t1;
        lineEndpoints[2*i] = wxRealPoint((t2 * startPoint.x + t1 * endPoint.x), (t2 * startPoint.y + t1 * endPoint.y));
        lineEndpoints[2*i+1] = wxRealPoint((t1 * startPoint.x + t2 * endPoint.x), (t1 * startPoint.y + t2 * endPoint.y));
    }

    path.MoveToPoint(lineEndpoints[0].x + diagonalLength,-(lineEndpoints[0].y - diagonalLength));
    path.AddLineToPoint(lineEndpoints[1].x + diagonalLength,-(lineEndpoints[1].y - diagonalLength));
    path.AddArcToPoint(corners[1].x + diagonalLength, -(corners[1].y - diagonalLength), lineEndpoints[2].x + diagonalLength, -(lineEndpoints[2].y - diagonalLength), cornerRadius);
    path.AddLineToPoint(lineEndpoints[3].x + diagonalLength,-(lineEndpoints[3].y - diagonalLength));
    path.AddArcToPoint(corners[2].x + diagonalLength, -(corners[2].y - diagonalLength), lineEndpoints[4].x + diagonalLength, -(lineEndpoints[4].y - diagonalLength), cornerRadius);
    path.AddLineToPoint(lineEndpoints[5].x + diagonalLength,-(lineEndpoints[5].y - diagonalLength));
    path.AddArcToPoint(corners[3].x + diagonalLength, -(corners[3].y - diagonalLength), lineEndpoints[6].x + diagonalLength, -(lineEndpoints[6].y - diagonalLength), cornerRadius);
    path.AddLineToPoint(lineEndpoints[7].x + diagonalLength,-(lineEndpoints[7].y - diagonalLength));
    path.AddArcToPoint(corners[0].x + diagonalLength, -(corners[0].y - diagonalLength), lineEndpoints[0].x + diagonalLength, -(lineEndpoints[0].y - diagonalLength), cornerRadius);
    path.CloseSubpath();
    gc->SetBrush(gc->CreateBrush(wxBrush(wxColour("#2b2b2b")))); // fill color
    gc->DrawPath(path);
    return path;
}

wxRealPoint Helper::rotatePointByDegrees(wxRealPoint point, double degrees){
    double radians = Helper::degreesToRadians(degrees);
    return Helper::rotatePointByRadians(point, radians);
}

wxRealPoint Helper::rotatePointByRadians(wxRealPoint point, double radians){
    return {
        point.x * cos(radians) - point.y * sin(radians),
        point.y * cos(radians) + point.x * sin(radians)
    };
}

double Helper::crossProduct(wxRealPoint p1, wxRealPoint p2){
    return (p1.x * p2.y) - (p1.y * p2.x);
}

int Helper::sign(double val){
    if(val < 0){
        return -1;
    }
    return 1;
}

wxRealPoint Helper::toMeters(wxRealPoint pointPixels){
    return {pointPixels.x / Helper::PIXELS_PER_METER, pointPixels.y / Helper::PIXELS_PER_METER};
}

wxRealPoint Helper::toPixels(wxRealPoint pointMeters){
    return {pointMeters.x * Helper::PIXELS_PER_METER, pointMeters.y * Helper::PIXELS_PER_METER};
}