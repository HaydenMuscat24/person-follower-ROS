/*
 * File         : controls.h
 * Author       : Timothy Wiley
 * Version      :
 * Created      : 03/10/2014
 * Description  : GUI Controls
 * Copyright    :
 */

#ifndef COMP3431_STARTER_RVIZ_CONTROLS_H_
#define COMP3431_STARTER_RVIZ_CONTROLS_H_

#include <ros/ros.h>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/property.h>

#include <crosbot/handle.hpp>
#include <crosbot/controls/command.hpp>
#include <crosbot/controls/status.hpp>

#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace comp3431 {

class ControlsWidget : public QWidget {
Q_OBJECT
public:
    ControlsWidget(rviz::Property* propertyParent);
    ~ControlsWidget() {};

    void start();
    void stop();

    // Save in RViz Config
    void loadRVizConfig(const rviz::Config& config);
    void saveRVizConfig(rviz::Config config) const;

    // Subscriber callbacks
    virtual void callback_receivedStatus(const crosbot_msgs::ControlStatusPtr status);
    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command);

public Q_SLOTS:
    void slot_btn_reset();
    void slot_btn_start();
    void slot_btn_stop();

Q_SIGNALS:
    //void signal_update_data();

private:
    // Widget active
    bool operating;

    // ROS elements
    ros::Subscriber subStatus;

    // Crosbot controls
    crosbot::CrosbotCommandPtr crosbotCommand;
    crosbot::CrosbotStatusPtr crosbotStatus;

    // RViz Properties
    rviz::Property* propertyParent;

    // Status
    QLabel *commandLabel;
    QLabel *statusLabel;

    // Control elements
    QPushButton *resetBtn;
    QPushButton *startBtn;
    QPushButton *stopBtn;
};

class ControlsRVizDisplay : public rviz::Display {
Q_OBJECT
public:
    ControlsRVizDisplay();
    ~ControlsRVizDisplay();

    // Over-ridden methods
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

protected:
   virtual void onInitialize();
   virtual void reset();
   virtual void onDisable();
   virtual void onEnable();

private:
    // Widget rendering the controls
    ControlsWidget* controlsWidget;
    rviz::PanelDockWidget* controlsWidgetContainer;
};
typedef crosbot::Handle<ControlsWidget> ControlsWidgetPtr;

} // namespace comp3431

#endif /* COMP3431_STARTER_RVIZ_CONTROLS_H_ */
