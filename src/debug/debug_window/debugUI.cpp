#include "debugUI.h"

#define VNAME(x) #x

debugUI::debugUI(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade): 
Gtk::Window(cobject), builder(refGlade)
// , mySpin(1,&myQueue)
{
    linkWidgets();
    first = true;
    firstTimeStamp = 0.0f;
    std::string logTopic = myDrone.name + "/log";
    // logSubscriber = myNode.subscribe<multi_drone_platform::droneLog>(logTopic, 10,&debugUI::logCallback, this);

    // myNode.setCallbackQueue(&myQueue);
    
}
void debugUI::init(mdp_api::id droneName, std::array<int, 2> startLocation, bool expanded)
{
    this->expanded = false;
    if (expanded)
    {
        on_expandButton_clicked();
        // this->expanded = true;
    }
    if (startLocation != std::array<int,2>({0, 0}))
    {
        this->move(startLocation[0], startLocation[1]);
    }
    myDrone = droneName;
    this->set_title(myDrone.name);
    droneNameLabel->set_label(myDrone.name);
    speedScale->set_value(1.0);
    this->show();
}

void debugUI::updateStats()
{
    
}

void debugUI::on_landButton_clicked()
{
    
}
void debugUI::on_emergencyButton_clicked()
{

}

void debugUI::logCallback(const multi_drone_platform::droneLog::ConstPtr& msg)
{
    if (first)
    {
        first = false;
        firstTimeStamp = msg->timeStamp;
        logTextBuffer->set_text("");
    }
    float time = msg->timeStamp - firstTimeStamp;

    std::ostringstream streamObj;
	streamObj << std::fixed;
	streamObj << std::setprecision(4);
	streamObj << time;

    
    std::string newLogLine = streamObj.str() + ": " + logType[msg->type] + " " + msg->logMessage + "\n";

    logTextBuffer->insert(logTextBuffer->end(), newLogLine);

}

void debugUI::on_speedScale_value_changed()
{
    std::ostringstream streamObj;
	streamObj << std::fixed;
	streamObj << std::setprecision(2);
	streamObj << speedScale->get_value();
  
    speedMultiplierLabel->set_label(streamObj.str());
}
void debugUI::on_expandButton_clicked()
{
    sidePanelGrid->set_visible(!expanded);
    logScroll->set_visible(!expanded);
    expandButton->set_image(expanded ? *expandImage : *compressImage);
    this->resize(1, 1);
    expanded = !expanded;
}

void debugUI::on_debugWindow_destroy()
{
    std::cout<<"TCHUSSSS"<<std::endl;
}

void debugUI::linkWidgets()
{
    try
    {
        builder->get_widget(VNAME(droneNameLabel), droneNameLabel);
        builder->get_widget(VNAME(droneNameLabel), droneNameLabel);
        builder->get_widget(VNAME(currPosX), currPosX);
        builder->get_widget(VNAME(currPosY), currPosY);
        builder->get_widget(VNAME(currPosZ), currPosZ);
        builder->get_widget(VNAME(currYaw), currYaw);
        builder->get_widget(VNAME(currVelX), currVelX);
        builder->get_widget(VNAME(currVelY), currVelY);
        builder->get_widget(VNAME(currVelZ), currVelZ);
        builder->get_widget(VNAME(currYawRate), currYawRate);
        builder->get_widget(VNAME(desPosX), desPosX);
        builder->get_widget(VNAME(desPosY), desPosY);
        builder->get_widget(VNAME(desPosZ), desPosZ);
        builder->get_widget(VNAME(desYaw), desYaw);
        builder->get_widget(VNAME(desVelX), desVelX);
        builder->get_widget(VNAME(desVelY), desVelY);
        builder->get_widget(VNAME(desVelZ), desVelZ);
        builder->get_widget(VNAME(desYawRate), desYawRate);
        builder->get_widget(VNAME(stateInput), stateInput);
        builder->get_widget(VNAME(queueInput), queueInput);
        builder->get_widget(VNAME(batteryInput), batteryInput);
        builder->get_widget(VNAME(batteryLevelBar), batteryLevelBar);
        builder->get_widget(VNAME(speedMultiplierLabel), speedMultiplierLabel);
        builder->get_widget(VNAME(pktLossLabel), pktLossLabel);
        builder->get_widget(VNAME(logTextView), logTextView);
        // builder->get_widget(VNAME(logTextBuffer), logTextBuffer);
        builder->get_widget(VNAME(landButton), landButton);
        builder->get_widget(VNAME(emergencyButton), emergencyButton);
        builder->get_widget(VNAME(speedScale), speedScale);
        builder->get_widget(VNAME(expandButton), expandButton);
        builder->get_widget(VNAME(sidePanelGrid), sidePanelGrid);
        builder->get_widget(VNAME(droneStatTable), droneStatTable);
        builder->get_widget(VNAME(topViewBottom), topViewBottom);
        builder->get_widget(VNAME(topViewTop), topViewTop);
        builder->get_widget(VNAME(topViewLeft), topViewLeft);
        builder->get_widget(VNAME(topViewRight), topViewRight);
        builder->get_widget(VNAME(topViewTopLeft), topViewTopLeft);
        builder->get_widget(VNAME(topViewTopRight), topViewTopRight);
        builder->get_widget(VNAME(topViewBotRight), topViewBotRight);
        builder->get_widget(VNAME(topViewBotLeft), topViewBotLeft);
        builder->get_widget(VNAME(sideViewTop), sideViewTop);
        builder->get_widget(VNAME(sideViewBottom), sideViewBottom);
        builder->get_widget(VNAME(logScroll),logScroll);
        builder->get_widget(VNAME(compressImage), compressImage);
        builder->get_widget(VNAME(expandImage), expandImage);


        landButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debugUI::on_landButton_clicked));
        
        emergencyButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debugUI::on_emergencyButton_clicked));
        
        speedScale->signal_value_changed().connect
        (sigc::mem_fun(*this, &debugUI::on_speedScale_value_changed));

        expandButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debugUI::on_expandButton_clicked));

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }   
}
