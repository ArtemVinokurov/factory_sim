// File:          robot_control.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          3

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
    // ********************************************************************
    // Variable declarations and definitions

    int                         ResultValue                 =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;

    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************

    printf("-------------------------------------------------------\n"  );
    printf("Reflexxes Motion Libraries                             \n"  );
    printf("Example: 01_RMLPositionSampleApplication               \n\n");
    printf("This example demonstrates the most basic use of the    \n"  );
    printf("Reflexxes API (class ReflexxesAPI) using the position- \n"  );
    printf("based Type II Online Trajectory Generation algorithm.  \n\n");
    printf("Copyright (C) 2014 Google, Inc.                      \n"  );
    printf("-------------------------------------------------------\n"  );

    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLPositionInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.

    IP->CurrentPositionVector->VecData      [0] =    100.0      ;
    IP->CurrentPositionVector->VecData      [1] =      0.0      ;
    IP->CurrentPositionVector->VecData      [2] =     50.0      ;

    IP->CurrentVelocityVector->VecData      [0] =    100.0      ;
    IP->CurrentVelocityVector->VecData      [1] =   -220.0      ;
    IP->CurrentVelocityVector->VecData      [2] =    -50.0      ;

    IP->CurrentAccelerationVector->VecData  [0] =   -150.0      ;
    IP->CurrentAccelerationVector->VecData  [1] =    250.0      ;
    IP->CurrentAccelerationVector->VecData  [2] =    -50.0      ;

    IP->MaxVelocityVector->VecData          [0] =    300.0      ;
    IP->MaxVelocityVector->VecData          [1] =    100.0      ;
    IP->MaxVelocityVector->VecData          [2] =    300.0      ;

    IP->MaxAccelerationVector->VecData      [0] =    300.0      ;
    IP->MaxAccelerationVector->VecData      [1] =    200.0      ;
    IP->MaxAccelerationVector->VecData      [2] =    100.0      ;

    IP->MaxJerkVector->VecData              [0] =    400.0      ;
    IP->MaxJerkVector->VecData              [1] =    300.0      ;
    IP->MaxJerkVector->VecData              [2] =    200.0      ;

    IP->TargetPositionVector->VecData       [0] =   -600.0      ;
    IP->TargetPositionVector->VecData       [1] =   -200.0      ;
    IP->TargetPositionVector->VecData       [2] =   -350.0      ;

    IP->TargetVelocityVector->VecData       [0] =    50.0       ;
    IP->TargetVelocityVector->VecData       [1] =   -50.0       ;
    IP->TargetVelocityVector->VecData       [2] =  -200.0       ;

    IP->SelectionVector->VecData            [0] =   true        ;
    IP->SelectionVector->VecData            [1] =   true        ;
    IP->SelectionVector->VecData            [2] =   true        ;

    // ********************************************************************
    // Starting the control loop

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        // ****************************************************************
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************

        // Calling the Reflexxes OTG algorithm
        ResultValue =   RML->RMLPosition(       *IP
                                            ,   OP
                                            ,   Flags       );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        // ****************************************************************
        // Here, the new state of motion, that is
        //
        // - OP->NewPositionVector
        // - OP->NewVelocityVector
        // - OP->NewAccelerationVector
        //
        // can be used as input values for lower level controllers. In the
        // most simple case, a position controller in actuator space is
        // used, but the computed state can be applied to many other
        // controllers (e.g., Cartesian impedance controllers,
        // operational space controllers).
        // ****************************************************************

        // ****************************************************************
        // Feed the output values of the current control cycle back to
        // input values of the next control cycle

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
    }

    // ********************************************************************
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process

    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
