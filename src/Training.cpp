#include "feedback_bci/Training.h"


namespace feedback {

Training::Training(void) : p_nh_("~") {
    this->pub_events_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
}

Training::~Training(void) {}

bool Training::configure(void) {

    this->name_ = "Training";
    // Getting modality and the paradigm
    std::string modality, paradigm, topic;
    if(this->p_nh_.getParam("modality", modality) == false) {
        ROS_ERROR("[%s] Parameter 'modality' is mandatory", this->name_.c_str());
        return false;
    }
    if(this->p_nh_.getParam("paradigm", paradigm) == false) {
        ROS_ERROR("[%s] Parameter 'paradigm' is mandatory", this->name_.c_str());
        return false;
    }

    topic = "/" + paradigm + "/integrated/normalized";
    if(modality.compare("calibration") == 0) {
        this->modality_ = Modality::Calibration;
        this->pub_probs_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic, 1);
    } else if(modality.compare("evaluation") == 0) {
        this->sub_probs_ = this->nh_.subscribe(topic, 1, &Training::on_received_data, this);
        this->modality_ = Modality::Evaluation;
    } else {
        ROS_ERROR("[%s] Unknown modality provided", this->name_.c_str());
        return false;
    }

    // Getting classes and trials
    if(this->p_nh_.getParam("classes", this->classes_) == false) {
        ROS_ERROR("[%s] Parameter 'classes' is mandatory", this->name_.c_str());
        return false;
    } 
    this->nclasses_ = this->classes_.size();
    this->thresholds_ = std::vector<float>(this->nclasses_, 1.0f);

    if(this->p_nh_.getParam("trials", this->trials_per_class_) == false) {
        ROS_ERROR("[%s] Parameter 'trials' is mandatory", this->name_.c_str());
        return false;
    } else if(this->trials_per_class_.size() != this->nclasses_) { 
        ROS_ERROR("[%s] Number of trials per class must be provided for each class", this->name_.c_str());
        return false;
    }
    
    // Getting postive feedback parameter
    this->p_nh_.param("positive_feedback", this->positive_feedback_, false);
    ROS_WARN("[%s] Positive feedback is %s", this->name_.c_str(), this->positive_feedback_ ? "enabled" : "disabled");

    // Getting duration parameters
    ros::param::param("~duration/begin",            this->duration_.begin,             5000);
    ros::param::param("~duration/start",            this->duration_.start,             1000);
    ros::param::param("~duration/fixation",         this->duration_.fixation,          2000);
    ros::param::param("~duration/cue",              this->duration_.cue,               1000);
    ros::param::param("~duration/feedback_min",     this->duration_.feedback_min,      4000); // duration of cf
    ros::param::param("~duration/feedback_max",     this->duration_.feedback_max,      5500);
    ros::param::param("~duration/boom",             this->duration_.boom,              1500);
    ros::param::param("~duration/timeout",          this->duration_.timeout,          10000); // duration of cf
    ros::param::param("~duration/timeout_on_rest",  this->duration_.timeout_on_rest,   6000);
    ros::param::param("~duration/iti",              this->duration_.iti,                100);
    ros::param::param("~duration/end",              this->duration_.end,               2000);
    ros::param::param("~duration/calibration",      this->duration_.calibration,       2000);


    // Setting parameters
    if(this->modality_ == Modality::Calibration) {
        this->mindur_active_ = this->duration_.feedback_min;
        this->maxdur_active_ = this->duration_.feedback_max;
    } else {
        this->mindur_active_ = this->duration_.timeout;
        this->maxdur_active_ = this->duration_.timeout;
        this->mindur_rest_   = this->duration_.timeout_on_rest;
        this->maxdur_rest_   = this->duration_.timeout_on_rest;
    }

    for(int i = 0; i < this->nclasses_; i++) {
        if(this->classes_.at(i) == Events::Rest){
            this->trialsequence_.addclass(this->classes_.at(i), this->trials_per_class_.at(i), this->mindur_rest_, this->maxdur_rest_);
        }else{
            this->trialsequence_.addclass(this->classes_.at(i), this->trials_per_class_.at(i), this->mindur_active_, this->maxdur_active_);
        }
    }
    
    ROS_INFO("[%s] Total number of classes: %ld", this->name_.c_str(), this->classes_.size());
    ROS_INFO("[%s] Total number of trials:  %d", this->name_.c_str(), this->trialsequence_.size());
    ROS_INFO("[%s] Trials have been randomized", this->name_.c_str());

    return true;

}

int Training::class2direction(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    
    if(it != this->classes_.end())
        return int(it - this->classes_.begin());

    return -1;
}

int Training::class2index(int eventcue) {

    auto it = find(this->classes_.begin(), this->classes_.end(), eventcue);
    int idx = -1;

    if(it != this->classes_.end()){
        idx = std::distance(this->classes_.begin(), it);
    }else{
        ROS_ERROR("[%s] Class %d not found", this->name_.c_str(), eventcue);
    }

    return idx;
}

float Training::direction2threshold(int index) {

    if(index != -1) {
        return this->thresholds_[index];
    } else {
        ROS_ERROR("[%s] Unknown direction", this->name_.c_str());
        return -1;
    }
}

std::vector<std::vector<float>> Training::str2matrix(const std::string& str) {
    std::vector<std::vector<float>> matrix;
    std::istringstream iss(str);
    std::string row_str;
    while (std::getline(iss, row_str, ';')) {
        std::istringstream row_ss(row_str);
        float value;
        std::vector<float> row_vector;
        while (row_ss >> value) {
            row_vector.push_back(value);
        }
        matrix.push_back(row_vector);
    }

    return matrix;
}

void Training::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

    // Check if the incoming message has the provided classes
    bool class_not_found = false;
    std::vector<int> msgclasses = msg.decoder.classes;

    // Check that the incoming classes are the ones provided
    for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it) {
        auto it2 = std::find(this->classes_.begin(), this->classes_.end(), *it);
        if(it2 == this->classes_.end()) {
            class_not_found = true;
            break;
        }
    }

    if(class_not_found == true) {
        ROS_WARN_THROTTLE(5.0f, "[%s] The incoming neurooutput message does not have the provided classes", this->name_.c_str());
        return;
    }

    // Set the new incoming data
    this->current_input_ = msg.softpredict.data;

    //std::cout << "Received data: " << this->current_input_[0] << " " << this->current_input_[1] << std::endl;  
}

void Training::run(void){
    int                    trialnumber;
    int                    trialclass;
    int                    trialduration;
    float                  trialthreshold;
    int                    hitclass;
    int                    boomevent;
    int                    idx_class;
    int                    trialdirection;
    int                    trialhit;
    std::vector<int>       count_results = std::vector<int>(3, 0); // count hit, miss and timeout
    ros::Rate r(this->rate_);
    std::vector<int> idxs_classes(this->nclasses_);
    std::iota(idxs_classes.begin(), idxs_classes.end(), 0);
    std::vector<int> other_idx_classes;
    

    rosneuro::feedback::LinearPilot linearpilot(1000.0f/this->rate_);
    rosneuro::feedback::SinePilot   sinepilot(1000.0f/this->rate_, 0.25f, 0.5f);
    rosneuro::feedback::Autopilot*  autopilot;

    // Begin
    ROS_INFO("[%s] Protocol BCI started", this->name_.c_str());
    this->sleep(this->duration_.begin);

    // open the feedback audio, with the default values: all wav file have the same sample rate and channels
    this->openAudioDevice();
    
    for(int i = 0; i < this->trialsequence_.size(); i++) {
        // Getting trial information
        trialnumber    = i + 1;
        Trial t = this->trialsequence_.gettrial(i);
        trialclass     = t.classid;
        trialduration  = t.duration;
        
        if(this->fake_rest_ && this->modality_ == Modality::Calibration && trialclass == Events::Fake_rest){
            std::random_device rd; 
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dist(0, this->nclasses_-1);
            idx_class = dist(gen); 
            fake_trialclass = this->classes_.at(idx_class);
            trialdirection = this->class2direction(fake_trialclass);
            trialthreshold = this->direction2threshold(trialdirection);
            other_idx_classes = idxs_classes;
            other_idx_classes.erase(std::remove(other_idx_classes.begin(), other_idx_classes.end(), idx_class), other_idx_classes.end());
        }else{
            idx_class      = this->class2index(trialclass); 
            trialdirection = this->class2direction(trialclass);
            trialthreshold = this->direction2threshold(trialdirection);
        }
        trialhit      = -1;
        this->trial_ok_ = 1;

        if(this->modality_ == Modality::Calibration) {
            if(trialclass == Events::Fake_rest && this->fake_rest_){
                autopilot = &sinepilot;
            }else{
                autopilot = &linearpilot;
            }
            autopilot->set(0.0f, trialthreshold, trialduration);
        }

        ROS_INFO("[Training_CVSA] Trial %d/%d (class: %d | duration cf: %d ms)", trialnumber, this->trialsequence_.size(), trialclass, trialduration);
        this->setevent(Events::Start);
        this->sleep(this->duration_.start);
        //this->setevent(Events::Start + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;
        

        /* FIXATION */
        this->setevent(Events::Fixation);
        this->show_fixation();
        this->sleep(this->duration_.fixation);
        this->hide_fixation();
        this->setevent(Events::Fixation + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;


        /* CUE */
        this->show_center();
        int idx_sampleAudio;
        size_t sampleAudio, bufferAudioSize, n_sampleAudio;
        if(this->fake_rest_ && this->modality_ == Modality::Calibration && trialclass == Events::Fake_rest){
            this->setevent(trialclass);
            this->sleep(this->duration_.iti);
            this->setevent(fake_trialclass);
        }else{
            this->setevent(trialclass);
        }
        this->timer_.tic();
        int c_time;
        if(this->audio_cue_){
            if(this->fake_rest_ && this->modality_ == Modality::Calibration && trialclass == Events::Fake_rest){
                this->loadWAVFile(this->audio_path_ + "/" + std::to_string(fake_trialclass) + ".wav");
            }else{
                this->loadWAVFile(this->audio_path_ + "/" + std::to_string(trialclass) + ".wav");
            }
            this->setAudio(idx_sampleAudio, sampleAudio, bufferAudioSize, n_sampleAudio);
            while((idx_sampleAudio + n_sampleAudio) * this->channels_audio_ <= this->buffer_audio_full_.size()){
                this->fillAudioBuffer(idx_sampleAudio, n_sampleAudio, true);
                ao_play(this->device_audio_, reinterpret_cast<char*>(this->buffer_audio_played_.data()), bufferAudioSize * sizeof(short));
            }
            c_time = this->timer_.toc();
            if(this->duration_.cue - c_time > 0){
                ROS_INFO("[Training_CVSA] Cue added time: %d ms", c_time);
                this->sleep(this->duration_.cue - c_time);
            }
        }else{
            this->show_cue(trialdirection);
            this->sleep(this->duration_.cue);
            this->hide_cue();
        }
        if(this->fake_rest_ && this->modality_ == Modality::Calibration && trialclass == Events::Fake_rest){
            this->sleep(this->duration_.iti);
            this->setevent(fake_trialclass + Events::Off);
        }
        this->setevent(trialclass + Events::Off);
        
        if(ros::ok() == false || this->user_quit_ == true) break;


        /* CONTINUOUS FEEDBACK */
        this->timer_.tic();

        // Consuming old messages
        ros::spinOnce();

        // Send cf event
        this->setevent(Events::CFeedback);

        // Start the sound feedback
        this->loadWAVFile(this->audio_path_ + "/" + this->audio_name_cf_);
        this->setAudio(idx_sampleAudio, sampleAudio, bufferAudioSize, n_sampleAudio);

        // Set up initial probabilities
        this->current_input_ = std::vector<float>(this->nactiveclasses_, 0.0f); 

        while(ros::ok() && this->user_quit_ == false && trialhit == -1 && idx_sampleAudio + n_sampleAudio < this->buffer_audio_full_.size()) {

            c_time = this->timer_.toc();
            if(this->modality_ == Modality::Calibration) {
                this->fillAudioBuffer(idx_sampleAudio, n_sampleAudio, false);
                ao_play(this->device_audio_, reinterpret_cast<char*>(this->buffer_audio_played_.data()), bufferAudioSize * sizeof(short));
                if(trialclass == Events::Fake_rest && this->fake_rest_){
                    float step = autopilot->step();
                    if((step <= 0 && this->current_input_[idx_class] == 0.0f && this->current_input_[other_idx_classes[0]] == 0.0f) ||
                       (step >= 0 && this->current_input_[other_idx_classes[0]] > 0.0f) ||
                       (step <= 0 && this->current_input_[idx_class] == 0.0f)){
                        this->current_input_[other_idx_classes[0]] = this->current_input_[other_idx_classes[0]] + (-1)*step; 
                    }else{
                        this->current_input_[idx_class] = this->current_input_[idx_class] + step;
                    }
                    //ROS_INFO("Probabilities: %f %f Thresholds: %f %f", this->current_input_[0], this->current_input_[1], this->thresholds_[0], this->thresholds_[1]);
                }else{
                    this->current_input_[idx_class] = this->current_input_[idx_class] + autopilot->step();
                }
            } else if(this->modality_ == Modality::Evaluation) {
                if(!this->positive_feedback_){
                    this->fillAudioBuffer(idx_sampleAudio, n_sampleAudio, false);
                    ao_play(this->device_audio_, reinterpret_cast<char*>(this->buffer_audio_played_.data()), bufferAudioSize * sizeof(short));
                }else{
                    std::vector<float> input_norm = this->normalize4audio(this->current_input_);
                    auto maxElemIter = std::max_element(input_norm.begin(), input_norm.end());
                    int idx_maxElem = std::distance(input_norm.begin(), maxElemIter);
                    if(idx_maxElem == idx_class){
                        this->fillAudioBuffer(idx_sampleAudio, n_sampleAudio, false);
                        ao_play(this->device_audio_, reinterpret_cast<char*>(this->buffer_audio_played_.data()), bufferAudioSize * sizeof(short));
                    }
                }
                //ROS_INFO("Probabilities: %f %f Thresholds: %f %f", this->current_input_[0], this->current_input_[1], this->thresholds_[0], this->thresholds_[1]);
            }

            trialhit = this->is_target_hit(this->current_input_,  
                                            c_time, trialduration);

            if(trialhit != -1)
                break;
        
            r.sleep();
            ros::spinOnce();
        }
        this->play_fadeout(idx_sampleAudio, n_sampleAudio, bufferAudioSize, false);
        this->setevent(Events::CFeedback + Events::Off);
        if(ros::ok() == false || this->user_quit_ == true) break;
        

        /* BOOM */
        if(trialdirection == trialhit){
            boomevent = Events::Hit;
        }else if(trialhit >= 0 && trialhit < this->nactiveclasses_){
            boomevent = Events::Miss;
        }else{
            if(trialclass != Events::Rest){
                boomevent = Events::Timeout;
            }else{
                boomevent = Events::Hit; // consider a hit if the rest class is presented and timeout occurs
            }
        }
        // for the robot motion
        if(this->robot_control_){
            this->setevent(boomevent);
            this->show_boom(trialdirection, trialhit);
            this->timer_.tic();
            std_srvs::Trigger srv;
            while(true){
                this->srv_robot_moving_.call(srv.request, srv.response);
                if(!srv.response.success){
                    break;
                }else{
                    ROS_WARN_ONCE("[Training_CVSA] Robot is moving. Waiting for the robot to stop.");
                }
                this->sleep(500);
            }
            c_time = this->timer_.toc();
            if(c_time < this->duration_.boom){
                this->sleep(this->duration_.boom - c_time);
                ROS_INFO("[Training_CVSA] Boom added time: %d ms", c_time);
            }
            this->hide_boom();
            this->setevent(boomevent + Events::Off);
        }else{
            std::cout << "trial direction: " << trialdirection << " trial hit: " << trialhit << std::endl;
            this->setevent(boomevent);
            if(trialclass == Events::Rest){
                this->show_center_rest(trialhit);
                this->sleep(this->duration_.boom);
                this->hide_center_rest();
            }else{
                this->show_boom(trialdirection, trialhit);
                this->sleep(this->duration_.boom);
                this->hide_boom();
            }
            
            this->setevent(boomevent + Events::Off);
        }

        switch(boomevent) {
            case Events::Hit:
                count_results[0] = count_results[0]+1;
                ROS_INFO("[Training_CVSA] Target hit");
                break;
            case Events::Miss:
                count_results[1] = count_results[1]+1;
                ROS_INFO("[Training_CVSA] Target miss");
                break;
            case Events::Timeout:
                count_results[2] = count_results[2]+1;
                ROS_INFO("[Training_CVSA] Timeout reached. Time elapsed: %d, time duration: %d", c_time, trialduration);
                break;
        }


        /* FINISH the trial */
        this->hide_center();
        this->setevent(Events::Start + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;
        this->trials_keep_.push_back(this->trial_ok_);

        // Inter trial interval
        this->reset();
        this->sleep(this->duration_.iti);

        if(ros::ok() == false || this->user_quit_ == true) break;

    }

    // close the audio device
    this->closeAudioDevice();

    // Print accuracy
    ROS_INFO("[Training_CVSA] Hit: %d, Miss: %d, Timeout: %d", count_results[0], count_results[1], count_results[2]);

    // End
    if(this->user_quit_ == false)
        this->sleep(this->duration_.end);
    ROS_INFO("[Training_CVSA] Protocol ended");

    // Publish the trials keep
    if(this->eye_motion_online_){
        feedback_bci::Trials_to_keep msg;
        msg.trials_to_keep = this->trials_keep_;
        this->pub_trials_keep_.publish(msg);
    }
}


void TrainingCVSA::setevent(int event) {

    this->event_msg_.header.stamp = ros::Time::now();
    this->event_msg_.event = event;
    this->pub_.publish(this->event_msg_);
}

void TrainingCVSA::sleep(int msecs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

int TrainingCVSA::is_target_hit(std::vector<float> input, int elapsed, int duration) {

    int target = -1;

    for(int i = 0; i < this->nclasses_; i++) {
        if(input.at(i) >= this->thresholds_.at(i)) { 
            target = i;
            break;
        } else if(elapsed > duration){
            target = CuePalette.size()-1;
            break; 
        }
    }
    
    return target;
}


} // namespace feedback