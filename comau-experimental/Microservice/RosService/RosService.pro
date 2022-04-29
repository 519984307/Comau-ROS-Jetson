QT -= gui
QT += core sql network widgets
CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        DAL/RosActionDAL.cpp \
        ROSPlan.cpp \
        RosAction.cpp \
        RosActionService.cpp \
        main.cpp \
        rosnode.cpp \




# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -L$$PWD/../build-qwebappsocketupgrade-Desktop-Debug/ -lqwebappsocketupgrade

INCLUDEPATH += $$PWD/../build-qwebappsocketupgrade-Desktop-Debug \
                ../qwebappsocketupgrade
DEPENDPATH += $$PWD/../build-qwebappsocketupgrade-Desktop-Debug

unix:!macx: LIBS += -L$$PWD/../build-dbmanager-Desktop-Debug/ -ldbmanager

INCLUDEPATH += $$PWD/../build-dbmanager-Desktop-Debug \
                ../dbmanager
DEPENDPATH += $$PWD/../build-dbmanager-Desktop-Debug

unix:!macx: LIBS += -L$$PWD/../build-action-Desktop-Debug/ -laction

INCLUDEPATH += $$PWD/../build-action-Desktop-Debug \
                ../action
DEPENDPATH += $$PWD/../build-action-Desktop-Debug

HEADERS += \
    DAL/RosActionDAL.h \
    ROSPlan.h \
    RosAction.h \
    RosActionService.h \
    rosnode.h \
    utils.h \
    comau_msgs/ActionFeedback.h \
    comau_msgs/ActionRequest.h \
    comau_msgs/ActionResult.h \
    comau_msgs/ActionResultStatusConstants.h \
    comau_msgs/AddObject.h \
    comau_msgs/AddObjectRequest.h \
    comau_msgs/AddObjectResponse.h \
    comau_msgs/CartesianPose.h \
    comau_msgs/CartesianPoseStamped.h \
    comau_msgs/ComauRobotStatus.h \
    comau_msgs/ExecuteCartesianTrajectoryAction.h \
    comau_msgs/ExecuteCartesianTrajectoryActionFeedback.h \
    comau_msgs/ExecuteCartesianTrajectoryActionGoal.h \
    comau_msgs/ExecuteCartesianTrajectoryActionResult.h \
    comau_msgs/ExecuteCartesianTrajectoryFeedback.h \
    comau_msgs/ExecuteCartesianTrajectoryGoal.h \
    comau_msgs/ExecuteCartesianTrajectoryResult.h \
    comau_msgs/ExecuteJointTrajectoryAction.h \
    comau_msgs/ExecuteJointTrajectoryActionFeedback.h \
    comau_msgs/ExecuteJointTrajectoryActionGoal.h \
    comau_msgs/ExecuteJointTrajectoryActionResult.h \
    comau_msgs/ExecuteJointTrajectoryFeedback.h \
    comau_msgs/ExecuteJointTrajectoryGoal.h \
    comau_msgs/ExecuteJointTrajectoryResult.h \
    comau_msgs/GripperCommand.h \
    comau_msgs/GripperCommandRequest.h \
    comau_msgs/GripperCommandResponse.h \
    comau_msgs/MoveToJointsMoveItAction.h \
    comau_msgs/MoveToJointsMoveItActionFeedback.h \
    comau_msgs/MoveToJointsMoveItActionGoal.h \
    comau_msgs/MoveToJointsMoveItActionResult.h \
    comau_msgs/MoveToJointsMoveItFeedback.h \
    comau_msgs/MoveToJointsMoveItGoal.h \
    comau_msgs/MoveToJointsMoveItResult.h \
    comau_msgs/MoveToPoseMoveItAction.h \
    comau_msgs/MoveToPoseMoveItActionFeedback.h \
    comau_msgs/MoveToPoseMoveItActionGoal.h \
    comau_msgs/MoveToPoseMoveItActionResult.h \
    comau_msgs/MoveToPoseMoveItFeedback.h \
    comau_msgs/MoveToPoseMoveItGoal.h \
    comau_msgs/MoveToPoseMoveItResult.h \
    comau_msgs/PerceptionAction.h \
    comau_msgs/PerceptionActionFeedback.h \
    comau_msgs/PerceptionActionGoal.h \
    comau_msgs/PerceptionActionResult.h \
    comau_msgs/PerceptionFeedback.h \
    comau_msgs/PerceptionGoal.h \
    comau_msgs/PerceptionResult.h \
    comau_msgs/PropertyValuePair.h \
    comau_msgs/SnsTrkPlot.h \
    comau_msgs/SnsTrkPlotReset.h \
    comau_msgs/SnsTrkPlotResetRequest.h \
    comau_msgs/SnsTrkPlotResetResponse.h \




# Avoid using LD_LIBRARY_PATH=mylib ./RosService when launching the application
QMAKE_LFLAGS += -Wl,-rpath,"./mylib"

#INCLUDEPATH += /opt/ros/noetic/include
INCLUDEPATH += /opt/ros/$$(ROS_DISTRO)/include
#INCLUDEPATH += /home/antonio/catkin_ws/devel/include

LIBS += -L/opt/ros/$$(ROS_DISTRO)/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime -lrviz -lactionlib
LIBS += -lboost_system -lboost_thread


