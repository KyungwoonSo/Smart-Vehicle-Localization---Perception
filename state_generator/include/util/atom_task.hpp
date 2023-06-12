#pragma once
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <mutex>

class AtomTask{
    protected:
        AtomTask(int id, std::string task_name, double period);
        ~AtomTask();

    public:
        void Exec(int num_thread=4);
        void MainLoop();

    protected:
        virtual void Init()=0;
        virtual void Run()=0;
        virtual void Publish()=0;
        virtual void Terminate()=0;

    protected:
        std::string	  task_name_;
        double	      task_period_;   
        int		      task_rate_;
        ros::Time	  update_time_;
        ros::Duration duration_time_;
        ros::Duration execution_time_;
};

AtomTask::AtomTask(int id, std::string task_name, double period)
:task_name_(""),
task_period_(0.0),
task_rate_(0.0)
{
    task_name_ = task_name;
    task_period_ = period;
    task_rate_ = (int)(1.0/period);

    update_time_ = ros::Time::now();
    duration_time_ = ros::Duration(0.0);
    execution_time_ = ros::Duration(0.0);

    ros::NodeHandle nh;
}

AtomTask::~AtomTask(){
}

void AtomTask::Exec(int num_thread){
    boost::thread main_thread( boost::bind(  &AtomTask::MainLoop, this ) );

    ros::AsyncSpinner spinner(num_thread);
    spinner.start();
    ros::waitForShutdown();

    main_thread.join();
}

void AtomTask::MainLoop(){
    ros::Rate loop_rate(task_rate_);
    //ROS_INFO( "rate: %d", task_rate_ );

    Init();
    while( ros::ok() ){
        ros::Time time_now = ros::Time::now();
        duration_time_ = time_now - update_time_;
        update_time_ = time_now;
        
        Run();

        execution_time_ = ros::Time::now() - update_time_;

        Publish();    

        ros::spinOnce();
        loop_rate.sleep();
    }
    Terminate();
}