#include "feedback_bci/TrialSequence.h"

namespace feedback {


TrialSequence::TrialSequence(void) {

	this->rndgen_.seed(this->rnddev_());
}

TrialSequence::~TrialSequence(void) {}


void TrialSequence::create_trial_sequence(const std::vector<int>& classes, const std::vector<int>& trials) {

	this->nclasses_ = classes.size();
	this->ntrials_ = std::accumulate(trials.begin(), trials.end(), 0);

	for(auto i = 0; i < this->nclasses_; i++) {
		std::vector<int> index(trials.at(i), classes.at(i));
		this->trial_sequence_.insert(std::end(this->trial_sequence_), std::begin(index), std::end(index));
	}

	std::shuffle(std::begin(this->trial_sequence_), std::end(this->trial_sequence_), this->rndgen_);
}

void TrialSequence::create_duration_sequence(int mindur, int maxdur) {

	auto rnddis = std::uniform_int_distribution<int>(mindur, maxdur);
	
	for(auto i = 0; i < this->ntrials_; i++) 
		this->duration_sequence_.push_back(rnddis(this->rndgen_));

}

int TrialSequence::nclasses(void) {
	return this->nclasses_;
}

int TrialSequence::ntrials(void) {
	return this->ntrials_;
}


bool TrialSequence::addclass(int classid, int ntrials, int mindur, int maxdur) {

	auto is_already_present = [classid] (Trial i) { return i.classid == classid; };
	auto it = find_if(this->sequence_.begin(), this->sequence_.end(), is_already_present);

	if(it != this->sequence_.end())
		return false;

	Trial trial;
	auto intdis = std::uniform_int_distribution<int>(mindur, maxdur);
	trial.classid = classid;

	for(auto i = 0; i < ntrials; i++) {
		trial.duration = intdis(this->rndgen_); 
		this->sequence_.push_back(trial);
	}

	std::shuffle(std::begin(this->sequence_), std::end(this->sequence_), this->rndgen_);

	return true;
}

void TrialSequence::addtrial(int classid, int mindur, int maxdur) {

	auto intdis = std::uniform_int_distribution<int>(mindur, maxdur);

	Trial trial;
	trial.classid = classid;
	trial.duration = intdis(this->rndgen_);

	//std::cout<<"Class: "<<trial.classid<<" | Duration: "<<trial.duration<<std::endl;

	this->sequence_.push_back(trial);

}

Trial TrialSequence::gettrial(int idx) {
	return this->sequence_.at(idx);
}

bool TrialSequence::addclass(int classid, int ntrials, int dur) {
	return this->addclass(classid, ntrials, dur, dur);
}

void TrialSequence::dump(void) {

	for(auto it = this->sequence_.begin(); it != this->sequence_.end(); ++it)
		std::cout<<"Trial: "<<(it - this->sequence_.begin())<<" | Class: "<<(*it).classid<<" | Duration: "<<(*it).duration<<std::endl;

}

int TrialSequence::size(void) {
	return this->sequence_.size();
}

std::vector<Trial>::iterator TrialSequence::begin(void) {
	return this->sequence_.begin();
}

std::vector<Trial>::iterator TrialSequence::end(void) {
	return this->sequence_.end();
}

std::vector<Trial>::const_iterator TrialSequence::cbegin(void) const {
	return this->sequence_.cbegin();
}

std::vector<Trial>::const_iterator TrialSequence::cend(void) const {
	return this->sequence_.cend();
}




	}

