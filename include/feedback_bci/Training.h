#ifndef FEEDBACK_BCI_TRAININGCVSA_H_
#define FEEDBACK_BCI_TRAININGCVSA_H_

#include <numeric>
#include <array>
#include <ros/ros.h>
#include <random>

#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <neurochrono/Timer.h>

#include "feedback_bci/TrialSequence.h"
#include "feedback_bci/Autopilot.h"

#include <numeric>
#include <algorithm>

#include <sndfile.h>
#include <ao/ao.h>


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

    static const int Fake_rest     = 784;

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

    private:
        std::vector<std::vector<float>> str2matrix(const std::string& str);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Subscriber sub_probs_;
        ros::Publisher pub_events_;
        ros::Publisher pub_probs_;

        rosneuro_msgs::NeuroEvent  event_msg_;
        rosneuro_msgs::NeuroOutput inputmsg_;

        feedback::TrialSequence trialsequence_;

        std::vector<int> classes_;
        std::vector<int> trials_per_class_;
        std::vector<int> max_trials_per_class_;
        int nclasses_;
        std::string name_;

        Duration duration_;
        Modality modality_;
        int mindur_active_;
        int maxdur_active_;
        int mindur_rest_;
        int maxdur_rest_;

        // Timer
        neurochrono::timer_msecs timer_;

        std::vector<float> current_input_;
        const float rate_ = 100.0f;
        bool show_on_rest_;
        std::vector<float> thresholds_;
        std::vector<std::vector<float>> calibration_positions_;
        std::vector<int> calibration_classes_;
        int trial_ok_;
        

        // for positive feedback
        bool positive_feedback_ = false;

        // for robot control
        bool robot_control_ = false;

        // for fake rest
        bool fake_rest_ = false;

        // for imu
        bool imu_ = false;
};


}


#endif
