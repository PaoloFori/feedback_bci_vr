#include "feedback_bci/Training.h"


namespace feedback {

Training::Training(void) : p_nh_("~") {
    this->pub_events_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
}

Training::~Training(void) {}

bool Training::configure(void) {
    this->name_ = "Training";

    this->is_vr_ready_ = false;
    this->sub_vr_ready_ = this->nh_.subscribe("/vr/status/ready", 1, &Training::onVrReady, this);
    
    std::string modality, paradigm, topic_raw, topic_normalized;
    if(this->p_nh_.getParam("modality", modality) == false) {
        ROS_ERROR("[%s] Parameter 'modality' is mandatory", this->name_.c_str());
        return false;
    }
    if(this->p_nh_.getParam("paradigm", paradigm) == false) {
        ROS_ERROR("[%s] Parameter 'paradigm' is mandatory", this->name_.c_str());
        return false;
    }

    topic_raw = "/" + paradigm + "/neuroprediction/integrated/raw";
    topic_normalized = "/" + paradigm + "/neuroprediction/integrated/normalized";
    if(modality.compare("calibration") == 0) {
        this->modality_ = Modality::Calibration;
        this->pub_probs_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic_raw, 1);
    } else if(modality.compare("evaluation") == 0) {
        this->modality_ = Modality::Evaluation;
        this->sub_probs_ = this->nh_.subscribe(topic_raw, 1, &Training::on_received_data, this);
    } else {
        ROS_ERROR("[%s] Unknown modality provided", this->name_.c_str());
        return false;
    }
    this->pub_probs_norm_ = this->nh_.advertise<rosneuro_msgs::NeuroOutput>(topic_normalized, 1);

    // Getting classes and thresholds
    if(this->p_nh_.getParam("classes", this->classes_) == false) {
        ROS_ERROR("[%s] Parameter 'classes' is mandatory", this->name_.c_str());
        return false;
    } 
    this->nclasses_ = this->classes_.size();

    if(this->modality_ == Modality::Calibration){
        this->thresholds_ = std::vector<float>(this->nclasses_, 1.0f);
    }else{
        if(this->p_nh_.getParam("thresholds", this->thresholds_) == false) {
            ROS_ERROR("[%s] Parameter 'thresholds' is mandatory", this->name_.c_str());
            return false;
        } 
        if(this->thresholds_.size() != this->nclasses_){
            ROS_ERROR("[%s] The threshold per class must be provided for each class", this->name_.c_str());
            return false;
        }
    }


    if(this->p_nh_.getParam("trials", this->trials_per_class_) == false) {
        ROS_ERROR("[%s] Parameter 'trials' is mandatory", this->name_.c_str());
        return false;
    } else if(this->trials_per_class_.size() != this->nclasses_) { 
        ROS_ERROR("[%s] Number of trials per class must be provided for each class", this->name_.c_str());
        return false;
    }

    // Getting duration parameters
    ros::param::param("~duration/begin",            this->duration_.begin,             5000);
    ros::param::param("~duration/start",            this->duration_.start,             1000);
    ros::param::param("~duration/fixation",         this->duration_.fixation,          2000);
    ros::param::param("~duration/cue",              this->duration_.cue,               1500);
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
    rosneuro_msgs::NeuroOutput msg_copy;

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
    std::vector<float> fixed_input(this->nclasses_, 0.0f);
    for(size_t i = 0; i < msg.decoder.classes.size(); i++){
         int idx = this->class2index(msg.decoder.classes[i]);
         if(idx != -1) fixed_input[idx] = msg.softpredict.data[i];
    }
    this->current_input_ = fixed_input;

    std::vector<float> norm_probs = this->normalize_input(this->current_input_);
    msg_copy = msg;
    msg_copy.softpredict.data = norm_probs;
    this->pub_probs_norm_.publish(msg_copy);

    //std::cout << "Received data: " << this->current_input_[0] << " " << this->current_input_[1] << std::endl;  
}

void Training::onVrReady(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == true && !this->is_vr_ready_) {
        this->is_vr_ready_ = true;
        ROS_INFO("[%s] VR environment is ready. Starting the protocol.", this->name_.c_str());
    }
}

std::vector<float> Training::normalize_input(const std::vector<float>& input) {
    float p_rest = 1.0f / (float)this->classes_.size();
    std::vector<float> normalized_output(input.size(), p_rest);

    for (size_t i = 0; i < input.size(); ++i) {
        if (this->thresholds_[i] > p_rest) { 
                
            float slope = (1.0f - p_rest) / (this->thresholds_[i] - p_rest);

            float mapped_val = p_rest + (input[i] - p_rest) * slope;
            normalized_output[i] = std::max(0.0f, std::min(1.0f, mapped_val));

        } else {
            normalized_output[i] = input[i]; 
        }
    }
        
    return normalized_output;
}

void Training::setprobs(std::vector<float> probs) {
    rosneuro_msgs::NeuroOutput msg;

    msg.header.seq = this->seq_;
    msg.header.stamp = ros::Time::now();
    msg.decoder.classes = this->classes_;
    msg.softpredict.data = probs;
    std::vector<int> hardpredict(this->classes_.size(), 0);
    int max_idx = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));
    if(probs[max_idx] >= this->thresholds_[max_idx]) {
        hardpredict[max_idx] = 1;
    }
    msg.hardpredict.data = hardpredict;

    if(this->modality_==Modality::Calibration){
        this->pub_probs_.publish(msg);
    }

    std::vector<float> norm_probs = this->normalize_input(probs);
    msg.softpredict.data = norm_probs;
    this->pub_probs_norm_.publish(msg);
}


void Training::run(void){
    ROS_INFO("[%s] Waiting for VR environment to be ready...", this->name_.c_str());
    ros::Rate r(512);
    while(ros::ok() && this->is_vr_ready_ == false) {
        ros::spinOnce();
        r.sleep();
    }

    if(this->is_vr_ready_ == true) {
        this->bci_protocol();
    } else {
        ROS_WARN("[%s] VR environment is not ready. Protocol will not start.", this->name_.c_str());
    }
}

void Training::bci_protocol(void){
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
    
    for(int i = 0; i < this->trialsequence_.size(); i++) {
        // Getting trial information
        trialnumber    = i + 1;
        Trial t = this->trialsequence_.gettrial(i);
        trialclass     = t.classid;
        trialduration  = t.duration;
        idx_class      = this->class2index(trialclass); 
        trialdirection = this->class2direction(trialclass);
        trialthreshold = this->direction2threshold(trialdirection);
        trialhit      = -1;
        int c_time;

        if(this->modality_ == Modality::Calibration) {
            if(trialclass == Events::Rest){
                autopilot = &sinepilot;
            }else{
                autopilot = &linearpilot;
            }
            autopilot->set(0.0f, trialthreshold, trialduration);
        }

        ROS_INFO("[%s] Trial %d/%d (class: %d | duration cf: %d ms)", this->name_.c_str(), trialnumber, this->trialsequence_.size(), trialclass, trialduration);
        this->setevent(Events::Start);
        this->sleep(this->duration_.start);
        //this->setevent(Events::Start + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;
        
        /* FIXATION */
        this->setevent(Events::Fixation);
        this->sleep(this->duration_.fixation);
        this->setevent(Events::Fixation + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;


        /* CUE */
        this->setevent(trialclass);
        this->sleep(this->duration_.cue);
        this->setevent(trialclass + Events::Off);
        
        if(ros::ok() == false || this->user_quit_ == true) break;


        /* CONTINUOUS FEEDBACK */
        ros::spinOnce();

        // Send cf event
        this->setevent(Events::CFeedback);
        this->timer_.tic();
        this->current_input_ = std::vector<float>(this->nclasses_, 1.0f / this->nclasses_); 

        while(ros::ok() && this->user_quit_ == false && trialhit == -1) {

            c_time = this->timer_.toc();
            if(this->modality_ == Modality::Calibration) {
                this->current_input_[idx_class] = this->current_input_[idx_class] + autopilot->step() / (float) this->nclasses_;   
                this->setprobs(this->current_input_);
                this->seq_++; 
            }
            
            trialhit = this->is_target_hit(this->current_input_,  
                                            c_time, trialduration);

            if(trialhit != -1)
                break;
        
            r.sleep();
            ros::spinOnce();
        }
        this->setevent(Events::CFeedback + Events::Off);
        if(ros::ok() == false || this->user_quit_ == true) break;
        

        /* BOOM */
        if(trialdirection == trialhit){
            boomevent = Events::Hit;
        }else if(trialhit != this->nclasses_){
            boomevent = Events::Miss;
        }else{
            if(trialclass != Events::Rest){
                boomevent = Events::Timeout;
            }else{
                boomevent = Events::Hit; // consider a hit if the rest class is presented and timeout occurs
            }
        }
        
        // std::cout << "trial direction: " << trialdirection << " trial hit: " << trialhit << std::endl;
        this->setevent(boomevent);
        this->sleep(this->duration_.boom);
        this->setevent(boomevent + Events::Off);
        

        switch(boomevent) {
            case Events::Hit:
                count_results[0] = count_results[0]+1;
                ROS_INFO("[%s] Target hit", this->name_.c_str());
                break;
            case Events::Miss:
                count_results[1] = count_results[1]+1;
                ROS_INFO("[%s] Target miss", this->name_.c_str());
                break;
            case Events::Timeout:
                count_results[2] = count_results[2]+1;
                ROS_INFO("[%s] Timeout reached. Time elapsed: %d, time duration: %d", this->name_.c_str(), c_time, trialduration);
                break;
        }


        /* FINISH the trial */
        this->setevent(Events::Start + Events::Off);

        if(ros::ok() == false || this->user_quit_ == true) break;

        // Inter trial interval
        this->sleep(this->duration_.iti);

        if(ros::ok() == false || this->user_quit_ == true) break;

    }

    // Print accuracy
    ROS_INFO("[%s] Hit: %d, Miss: %d, Timeout: %d", this->name_.c_str(), count_results[0], count_results[1], count_results[2]);

    // End
    if(this->user_quit_ == false)
        this->sleep(this->duration_.end);
    ROS_INFO("[%s] Protocol ended", this->name_.c_str());
}


void Training::setevent(int event) {
    this->event_msg_.header.stamp = ros::Time::now();
    this->event_msg_.event = event;
    this->pub_events_.publish(this->event_msg_);
}

void Training::sleep(int msecs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
}

int Training::is_target_hit(std::vector<float> input, int elapsed, int duration) {
    for(int i = 0; i < this->nclasses_; i++) {
        if(input.at(i) >= this->thresholds_.at(i) - 0.001f) { 
            return i;
        }
    }
    
    if(elapsed > duration) {
        return this->nclasses_;
    }

    return -1;
}


} // namespace feedback