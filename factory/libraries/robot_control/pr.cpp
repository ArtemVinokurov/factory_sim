#include "pr.hpp"
#include <cmath>
#include <sstream>

static const double POSITION_TOLERANCE = 0.0002;

string PR::motorName(int motorIndex)
{
    switch(motorIndex) {
        case SHOULDER_PAN:
            return "pr15_shoulder_pan_joint";
        case SHOULDER_LIFT:
            return "pr15_shoulder_lift_joint";
        case ELBOW:
            return "pr15_elbow_joint";
        case WRIST_1:
            return "pr15_wrist1_joint";
        case WRIST_2:
            return "pr15_wrist2_joint";
        case WRIST_3:
            return "pr15_wrist3_joint";
    }
}

PR::PR() {
    mTimeStep = getBasicTimeStep();

    mMotors = new Motor *[NUMBER_OF_DOFS];

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        mMotors[i] = getMotor(motorName(i));
    }

    mPositionSensors = new PositionSensor *[NUMBER_OF_DOFS];

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        stringstream sensorName;
        sensorName << motorName(i) << "_sensor";
        mPositionSensors[i] = getPositionSensor(sensorName.str());
        mPositionSensors[i]->enable(mTimeStep);
    }

    RML = new RMLPositionInputParameters(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

}


