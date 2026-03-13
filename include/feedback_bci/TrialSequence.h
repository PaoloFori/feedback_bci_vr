#ifndef FEEDBACK_BCI_TRIALSQUENCE_H_
#define FEEDBACK_BCI_TRIALSQUENCE_H_

#include <iostream>
#include <numeric>
#include <random>
#include <algorithm>
#include <vector>

namespace feedback {

struct Trial {
	int classid;
	int duration;
};

class TrialSequence {

	public:
		TrialSequence(void);
		~TrialSequence(void);

		bool addclass(int classid, int ntrials, int mindur, int maxdur);
		bool addclass(int classid, int ntrials, int dur);
		
		int size(void);


		std::vector<Trial>::iterator begin(void);
		std::vector<Trial>::iterator end(void);
		std::vector<Trial>::const_iterator cbegin(void) const;
		std::vector<Trial>::const_iterator cend(void) const;

		void dump();

		int nclasses(void);
		int ntrials(void);

		void create_trial_sequence(const std::vector<int>& classes, const std::vector<int>& trials);
		void create_duration_sequence(int mindur, int maxdur);

		void addtrial(int classid,  int mindur, int maxdur);

		Trial gettrial(int idx);

	public:
		std::vector<int> trial_sequence_;
		std::vector<int> duration_sequence_;

	protected:
		int nclasses_;
		int ntrials_;

		std::vector<Trial> sequence_;
		int mindur_;
		int maxdur_;


		std::random_device rnddev_;
		std::mt19937 rndgen_;

};


}




#endif
