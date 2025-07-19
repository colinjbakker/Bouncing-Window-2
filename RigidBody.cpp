#include "RigidBody.h"
#include "Helper.h"

RigidBody::RigidBody(){

}

RigidBody::RigidBody(wxRealPoint startPosition, wxRealPoint startLinearVelocity, double startAngle, double startAngularVelocity, double diagonalLength, double mass, double moment){
    this->diagonalLength = diagonalLength;
    
    centerPosition = startPosition - wxRealPoint(diagonalLength/2, diagonalLength/2);
    linearVelocity = startLinearVelocity;

    rotationAngle = startAngle;
    angularVelocity = startAngularVelocity;
    
    this->mass = mass;
    momentOfInertia = moment;

    beingDragged = false;
    dragPivot = {0.0,0.0};
    currentMousePosition = {0.0,0.0};
    mouseVelocity = {0.0,0.0};
}

// dt in seconds
void RigidBody::Update(double dt){
    if(beingDragged){        
        rotationAngle += angularVelocity * dt;
        rotationAngle = fmod(rotationAngle, 360);

        wxRealPoint dragPivotRotated = Helper::rotatePointByDegrees(dragPivot, -rotationAngle);
        centerPosition = currentMousePosition - dragPivotRotated;
        
        //gravity pendulum motion
        wxRealPoint gravityImpulse = {0, Helper::GRAVITY * dt * mass};
        ApplyImpulse(gravityImpulse, Helper::toMeters(dragPivotRotated));
        
        //impulse from dragging
        wxRealPoint deltaV = (mouseVelocity - linearVelocity);
        ApplyImpulse(deltaV * mass, Helper::toMeters(-dragPivotRotated));
    } else {
        wxRealPoint gravityImpulse = {0, Helper::GRAVITY * dt * mass};
        ApplyImpulse(gravityImpulse, {0,0});
        // -315, -225, -135, -45, 45, 135, 225, 315
        RigidBody::PerformCollision();

        centerPosition += linearVelocity * Helper::PIXELS_PER_METER * dt;

        rotationAngle += angularVelocity * dt;
        rotationAngle = fmod(rotationAngle, 360);        
    }
    double linearDragFactor = std::exp(-Helper::AIR_RESISTANCE * dt);
    double angularDragFactor = std::exp(-Helper::AIR_RESISTANCE_ROTATION * dt);

    linearVelocity = {linearVelocity.x * linearDragFactor, linearVelocity.y * linearDragFactor};
    angularVelocity *= angularDragFactor;
}

//contact point in object coordinates, meters, (0,0) is center. impulse should be N*s or kg * m/s
void RigidBody::ApplyImpulse(wxRealPoint impulse, wxRealPoint contactPoint){
    linearVelocity.x += impulse.x / mass;
    linearVelocity.y += impulse.y / mass;

    double torque = Helper::crossProduct(contactPoint, impulse);
    angularVelocity += torque / momentOfInertia;
}

void RigidBody::SetDragged(bool dragging, wxRealPoint pivot){
    beingDragged = dragging;
    dragPivot = pivot;
    wxRealPoint dragPivotRotated = Helper::toMeters(Helper::rotatePointByDegrees(dragPivot, -rotationAngle));

    if(beingDragged){
        // on grabbing box
        ApplyImpulse(linearVelocity * mass, dragPivotRotated);
        linearVelocity = {0.0,0.0};
    } else {
        // on releasing box, throwing
        wxRealPoint releaseVelocity = wxRealPoint(-dragPivotRotated.y, dragPivotRotated.x) * Helper::degreesToRadians(angularVelocity);
        ApplyImpulse(releaseVelocity * mass, {0.0, 0.0});
    }
}

wxRect2DDouble RigidBody::ComputeAABB() {
    std::vector<wxRealPoint> corners = Helper::GetWorldCorners(rotationAngle, diagonalLength, centerPosition);
    double minx = corners[0].x;
    double maxx = corners[0].x;
    double miny = corners[0].y;
    double maxy = corners[0].y;

    for(auto &corner: corners){
        minx = std::min(minx, corner.x);
        maxx = std::max(maxx, corner.x);
        miny = std::min(miny, corner.y);
        maxy = std::max(maxy, corner.y);
    }

    double width = maxx - minx;
    double height = maxy - miny;
    wxRect2DDouble AABB = wxRect2DDouble(minx, miny, width, height);
    return AABB;
}

//TODO clean this up and add angular velocity effect for impulse. add friction perpendicular impulse.
void RigidBody::PerformCollision() {
    wxRect2DDouble AABB = this->ComputeAABB();
    double halfDiagonalLength = diagonalLength / 2;
    if (AABB.GetLeft() < 0) {
        centerPosition.x = AABB.GetSize().GetWidth() / 2 + 1;

        std::vector<wxRealPoint> corners = Helper::GetWorldCorners(rotationAngle, diagonalLength, centerPosition);
        wxRealPoint contact = corners[0];
        for (const auto& c : corners) {
            if (c.x < contact.x) contact = c;
        }

        wxRealPoint contactRel = Helper::toMeters({contact.x - centerPosition.x, contact.y - centerPosition.y});
        wxRealPoint vAtContact = linearVelocity + wxRealPoint(contactRel.y * Helper::degreesToRadians(angularVelocity), contactRel.x * Helper::degreesToRadians(angularVelocity));

        wxRealPoint bounceImpulse = {-(1 + Helper::RESTITUTION) * vAtContact.x * mass,0};

        double frictionMag = std::min(Helper::FRICTION * abs(bounceImpulse.x), abs(vAtContact.y) * mass);
        wxRealPoint frictionImpulse = {0.0, -frictionMag * Helper::sign(vAtContact.y)};
        
        ApplyImpulse(bounceImpulse, contactRel);
        ApplyImpulse(frictionImpulse, contactRel);
    }
    if (AABB.GetRight() > wxGetDisplaySize().GetWidth()) {
        centerPosition.x = wxGetDisplaySize().GetWidth() - AABB.GetSize().GetWidth() / 2 - 1;

        std::vector<wxRealPoint> corners = Helper::GetWorldCorners(rotationAngle, diagonalLength, centerPosition);
        wxRealPoint contact = corners[0];
        for (const auto& c : corners) {
            if (c.x > contact.x) contact = c;
        }
        
        wxRealPoint contactRel = Helper::toMeters({contact.x - centerPosition.x, contact.y - centerPosition.y});
        wxRealPoint vAtContact = linearVelocity + wxRealPoint(contactRel.y * Helper::degreesToRadians(angularVelocity), contactRel.x * Helper::degreesToRadians(angularVelocity));

        wxRealPoint bounceImpulse = {-(1 + Helper::RESTITUTION) * vAtContact.x * mass, 0};

        double frictionMag = std::min(Helper::FRICTION * abs(bounceImpulse.x), abs(vAtContact.y) * mass);
        wxRealPoint frictionImpulse = {0.0, -frictionMag * Helper::sign(vAtContact.y)};
        
        ApplyImpulse(bounceImpulse, contactRel);
        ApplyImpulse(frictionImpulse, contactRel);
    }
    if (AABB.GetTop() < 0) {
        centerPosition.y = AABB.GetSize().GetHeight() / 2 + 1;

        std::vector<wxRealPoint> corners = Helper::GetWorldCorners(rotationAngle, diagonalLength, centerPosition);
        wxRealPoint contact = corners[0];
        for (const auto& c : corners) {
            if (c.y < contact.y) contact = c;
        }
        
        wxRealPoint contactRel = Helper::toMeters({contact.x - centerPosition.x, contact.y - centerPosition.y});
        wxRealPoint vAtContact = linearVelocity + wxRealPoint(contactRel.y * Helper::degreesToRadians(angularVelocity), contactRel.x * Helper::degreesToRadians(angularVelocity));
        
        wxRealPoint bounceImpulse = {0, -(1 + Helper::RESTITUTION) * vAtContact.y * mass};
        
        double frictionMag = std::min(Helper::FRICTION * abs(bounceImpulse.y), abs(vAtContact.x) * mass);
        wxRealPoint frictionImpulse = {-frictionMag * Helper::sign(vAtContact.x), 0.0};
        
        ApplyImpulse(bounceImpulse, contactRel);
        ApplyImpulse(frictionImpulse, contactRel);
    }
    if (AABB.GetBottom() > wxGetDisplaySize().GetHeight()) {
        centerPosition.y = wxGetDisplaySize().GetHeight() - AABB.GetSize().GetHeight() / 2 - 1;

        std::vector<wxRealPoint> corners = Helper::GetWorldCorners(rotationAngle, diagonalLength, centerPosition);
        wxRealPoint contact = corners[0];
        for (const auto& c : corners) {
            if (c.y > contact.y) contact = c;
        }
        
        wxRealPoint contactRel = Helper::toMeters({contact.x - centerPosition.x, contact.y - centerPosition.y});
        wxRealPoint vAtContact = linearVelocity + wxRealPoint(contactRel.y * Helper::degreesToRadians(angularVelocity), contactRel.x * Helper::degreesToRadians(angularVelocity));
        
        wxRealPoint bounceImpulse = {0, -(1 + Helper::RESTITUTION) * vAtContact.y * mass};
        
        
        if(abs(fmod(abs(rotationAngle), 90.0) - 45.0) < 0.5 && abs(vAtContact.y) < 1.0 && abs(vAtContact.x) < 1.0 && abs(angularVelocity) < 10.0){
            angularVelocity = 0.0;
            linearVelocity = {0.0,0.0};
        } else {
            ApplyImpulse(bounceImpulse, contactRel);
        }
        double frictionMag = std::min(Helper::FRICTION * abs(bounceImpulse.y), abs(vAtContact.x) * mass);
        wxRealPoint frictionImpulse = {-frictionMag * Helper::sign(vAtContact.x), 0.0};
        
        
        ApplyImpulse(frictionImpulse, contactRel);
    }
}

wxRealPoint RigidBody::GetPosition(){
    return centerPosition;
}

double RigidBody::GetAngle(){
    return rotationAngle;
}

void RigidBody::SetPosition(wxRealPoint newPosition){
    centerPosition = newPosition;
}

void RigidBody::SetVelocity(wxRealPoint newVelocity){
    linearVelocity = Helper::toMeters(newVelocity);
}

void RigidBody::SetMouseVelocity(wxRealPoint newVelocity){
    mouseVelocity = Helper::toMeters(newVelocity);
}

void RigidBody::PushMousePosition(wxRealPoint newPosition){
    currentMousePosition = newPosition;
}
