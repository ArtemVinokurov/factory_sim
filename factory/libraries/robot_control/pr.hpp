#ifndef PR_HPP
#ifndef PR_HPP

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


using namespace webots;
using namespace std;
#define CYCLE_TIME_IN_SECONDS 0.001
#define NUMBER_OF_DOFS 6

class PR : public Robot {
    public:
      enum Motors {
        SHOULDER_PAN = 0,
        SHOULDER_LIFT = 1,
        ELBOW = 2,
        WRIST_1 = 3,
        WRIST_2 = 4,
        WRIST_3 = 5,
        };

        PR();
        virtual ~PR();

        double motorPosition(int motorIndex) const;
        bool positionReached(int motorIndex, double targetPosition) const;
        void setMotorPosition(int motorIndex, double position);
        void moveToInitPosition();
        void moveToPosition(const double *motorPositions, bool moveGripper = false);
        string motorName(int motorIndex);
        void moveJ()


        ReflexxesAPI *RML = NULL;
        RMLPositionInputParameters *IP = NULL;
        RMLPositionOutputParameters *OP = NULL;
        RMLPositionFlags Flags;
    
    private:
       int mTimeStep;
       Motor **mMotors;
       PositionSensor **mPositionSensors;
};

#endif
