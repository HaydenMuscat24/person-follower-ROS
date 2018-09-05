/*
 * File         : victimpanel.h
 * Author       : Timothy Wiley
 * Version      :
 * Created      : 09/02/2010
 * Description  : Victim Id Panel - based on existing SnapPanel
 * Copyright    :
 */

#include <ros/ros.h>

#include <comp3431_starter/common.hpp>
#include <comp3431_starter/rviz/controlPanel.hpp>

#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <typeinfo>
#include <QtCore/QVariant>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <utility>

#define COMMAND_LABEL           "Command: "
#define LOG_START               "ControlsWidget ::"
#define STATUS_LABEL            "Status: "

#define ADD_BUTTON(VAR, TEXT, LYT, ROW, COL, BSLOT)             \
        VAR = new QPushButton(TEXT);                            \
        VAR->setText(TEXT);                                     \
        VAR->setEnabled(true);                                  \
        LYT->addWidget(VAR, ROW, COL);                          \
        connect(VAR, SIGNAL(clicked()), this, SLOT(BSLOT()));

namespace comp3431 {

class _RVIZCommandCallback : public crosbot::CrosbotCommandCallback {
private:
    ControlsWidget *widget;

public:
    _RVIZCommandCallback(ControlsWidget *widget) : widget(widget) {};
    virtual ~_RVIZCommandCallback() {};
    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
        if (widget != NULL) widget->callback_receivedCommand(command);
    };
};

class _RVIZStatusCallback : public crosbot::CrosbotStatusCallback {
private:
    ControlsWidget *widget;

public:
    _RVIZStatusCallback(ControlsWidget *widget) : widget(widget) {};
    virtual ~_RVIZStatusCallback() {};
    virtual void callback_receivedStatus(const crosbot_msgs::ControlStatusPtr status) {
        if (widget != NULL) widget->callback_receivedStatus(status);
    };
};

ControlsWidget::ControlsWidget(rviz::Property* propertyParent) :
    QWidget(),
    operating(false),
    propertyParent(propertyParent)
{
    // Button Layout
    QVBoxLayout *vlayout = new QVBoxLayout(this);

    // Create crosbot buttons
    QGroupBox *controlsGB = new QGroupBox("Crosbot Controls");
    QGridLayout *crosbotButtonLayout = new QGridLayout();
    ADD_BUTTON(startBtn, "Start", crosbotButtonLayout, 0, 0, slot_btn_start);
    ADD_BUTTON(stopBtn, "Stop", crosbotButtonLayout, 0, 1, slot_btn_stop);
    ADD_BUTTON(resetBtn, "Reset", crosbotButtonLayout, 1, 0, slot_btn_reset);
    controlsGB->setLayout(crosbotButtonLayout);

    // Configure Labels
    QVBoxLayout *labelsLayout = new QVBoxLayout();
    commandLabel = new QLabel(COMMAND_LABEL, this);
    statusLabel = new QLabel(STATUS_LABEL, this);
    commandLabel->setWordWrap(true);
    statusLabel->setWordWrap(true);
    statusLabel->setText(STATUS_LABEL);
    labelsLayout->addWidget(commandLabel);
    labelsLayout->addWidget(statusLabel);

    // HRules
    QFrame *line = new QFrame(this);
    line->setGeometry(QRect(0, 0, 10, 3));
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    // Set layout
    vlayout->addWidget(controlsGB);
    vlayout->addWidget(line);
    vlayout->addItem(labelsLayout);
    setLayout(vlayout);
}

void ControlsWidget::start() {
    ROS_INFO("ControlsWidget::start");
    crosbotCommand = new crosbot::CrosbotCommand(TURTLEBOT_CONTROL_NAMESPACE,
            true,
            new _RVIZCommandCallback(this));
    crosbotStatus = new crosbot::CrosbotStatus("", new _RVIZStatusCallback(this));

    operating = true;
}

void ControlsWidget::stop() {
    ROS_INFO("ControlsWidget::stop");
    operating = false;

    crosbotCommand = NULL;
    //crosbotStatus = NULL;
}

void ControlsWidget::loadRVizConfig(const rviz::Config& config) {
}

void ControlsWidget::saveRVizConfig(rviz::Config config) const {
}

void ControlsWidget::callback_receivedStatus(const crosbot_msgs::ControlStatusPtr status) {
    if (operating) {
        // Set status
        std::string label = STATUS_LABEL;
        if (!status->stats_namespace.empty()) {
            label += "(" + status->stats_namespace + ")";
        }
        label += status->status;
        statusLabel->setText(label.c_str());

        // Set colour
        std::string level = status->level;
        QColor colour;
        if (level == crosbot_msgs::ControlStatus::LEVEL_INFO) {
            colour = Qt::black;
        } else if (level == crosbot_msgs::ControlStatus::LEVEL_WARNING) {
            colour = Qt::yellow;
        } else if (level == crosbot_msgs::ControlStatus::LEVEL_ERROR) {
            colour = Qt::red;
        }
        QPalette palette = statusLabel->palette();
        palette.setColor(statusLabel->foregroundRole(), colour);
        statusLabel->setPalette(palette);

        statusLabel->update();
    }
}

void ControlsWidget::callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
    if (operating) {
        std::string label = COMMAND_LABEL + command->command;
        commandLabel->setText(label.c_str());
        commandLabel->update();
    }
}

void ControlsWidget::slot_btn_reset() {
    if (operating) {
        ROS_INFO("%s send reset", LOG_START);
        crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_RESET);
    }
}

void ControlsWidget::slot_btn_start() {
    if (operating) {
        //ROS_INFO("%s Resume", LOG_START);
        crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_START);
    }
}

void ControlsWidget::slot_btn_stop() {
    if (operating) {
        //ROS_INFO("%s Stop", LOG_START);
        crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_STOP);
    }
}

ControlsRVizDisplay::ControlsRVizDisplay() :
    controlsWidget(NULL),
    controlsWidgetContainer(NULL)
{
    controlsWidget = new ControlsWidget(NULL);
}

ControlsRVizDisplay::~ControlsRVizDisplay() {
    if (controlsWidget != NULL) {
        ROS_INFO("ControlsRVizDisplay::~ControlsRVizDisplay");
        controlsWidget->stop();
        if (controlsWidgetContainer) {
            delete controlsWidgetContainer;
        } else {
            delete controlsWidget;
        }
    }
}

void ControlsRVizDisplay::load(const rviz::Config& config) {
    // Do Normal rviz::Display save
    rviz::Display::load(config);

    // Widgets load
    if (controlsWidget != NULL) {
        controlsWidget->loadRVizConfig(config);
    }
}

void ControlsRVizDisplay::save(rviz::Config config) const {
    // Do Normal rviz::Display save
    rviz::Display::save(config);

    // Widgets save
    if (controlsWidget != NULL) {
        controlsWidget->saveRVizConfig(config);
    }
}

void ControlsRVizDisplay::onInitialize() {
    // Connect widget to window manager and start
    ROS_INFO("ControlsRVizDisplay::onInitialize");
    rviz::WindowManagerInterface* wm = context_->getWindowManager();
    if (wm && controlsWidget != NULL) {
        // Controls widget
        controlsWidgetContainer = wm->addPane("Controls", controlsWidget);
    }
}

void ControlsRVizDisplay::reset() {
    ROS_INFO("ControlsRVizDisplay::reset");
    if (controlsWidget != NULL) {
        controlsWidget->stop();
        controlsWidget->start();
    }
}

void ControlsRVizDisplay::onDisable() {
    ROS_INFO("ControlsRVizDisplay::onDisable");
    if (controlsWidget != NULL) {
        controlsWidget->stop();
    }
}

void ControlsRVizDisplay::onEnable() {
    ROS_INFO("ControlsRVizDisplay::onEnable");
    if (controlsWidget != NULL) {
        controlsWidget->start();
    }
}

} // namespace comp3431

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(comp3431::ControlsRVizDisplay, rviz::Display)

