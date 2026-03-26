#ifndef FEEDBACK_BCI_TRAINING_H_
#define FEEDBACK_BCI_TRAINING_H_

#include <numeric>
#include <array>
#include <ros/ros.h>
#include <random>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <neurochrono/Timer.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>

#include "feedback_bci/TrialSequence.h"
#include "feedback_bci/Autopilot.h"

#include <numeric>
#include <algorithm>

#include <thread>


namespace feedback {

struct Events {
    static const int Start         = 1;
    static const int Fixation      = 786;
    static const int CFeedback     = 781;
    static const int Hit           = 897;
    static const int Miss          = 898;
    static const int Timeout       = 899;
    static const int Off           = 32768;
    static const int Rest          = 783;

    static const int StartCalibEye = 2;
    
};

struct Duration {
    int begin;
    int start;
    int fixation;
    int cue;
    int feedback_min;
    int feedback_max;
    int boom;
    int timeout;
    int timeout_on_rest;
    int iti;
    int end;
    int calibration;
};


class Training {

    public:
        enum class Modality {Calibration = 0, Evaluation};

    public:
        Training(void);
        virtual ~Training(void);
        virtual bool configure(void);
        virtual void run(void);

    protected:
        void setevent(int event);
        void sleep(int msecs);
        int class2direction(int eventcue);
        float direction2threshold(int index);
        int class2index(int eventcue);
        int is_target_hit(std::vector<float> input, int elapsed, int duration);
        void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
        void setprobs(std::vector<float> probs);
        void onVrReady(const std_msgs::Bool::ConstPtr& msg);
        void bci_protocol(void);

    private:
        std::vector<std::vector<float>> str2matrix(const std::string& str);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Subscriber sub_probs_;
        ros::Publisher pub_events_;
        ros::Publisher pub_probs_;
        ros::Subscriber sub_vr_ready_;

        std::string name_;

        rosneuro_msgs::NeuroEvent  event_msg_;
        rosneuro_msgs::NeuroOutput inputmsg_;

        feedback::TrialSequence trialsequence_;

        std::vector<int> classes_;
        std::vector<int> trials_per_class_;
        int nclasses_;
        
        Duration duration_;
        Modality modality_;
        int mindur_active_;
        int maxdur_active_;
        int mindur_rest_;
        int maxdur_rest_;

        neurochrono::timer_msecs timer_;

        std::vector<float> current_input_;
        const float rate_ = 100.0f;
        std::vector<float> thresholds_;
        bool user_quit_ = false;
        int seq_ = 0;

        bool is_vr_ready_;
};


}


#endif
