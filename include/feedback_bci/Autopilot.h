#ifndef ROSNEURO_FEEDBACK_AUTOPILOT_H_
#define ROSNEURO_FEEDBACK_AUTOPILOT_H_

#include <random>

namespace rosneuro {
	namespace feedback {


class Autopilot {

	public:
		Autopilot(float dt);
		virtual ~Autopilot(void);
		
		virtual void set(float start, float stop, int duration) = 0;
		virtual float step(void) = 0;

	protected:
		float dt_;
		std::random_device rnddev_;
		std::mt19937 rndgen_;
};

class LinearPilot : public Autopilot {

	public:
		LinearPilot(float dt);
		~LinearPilot(void);

		void set(float start, float stop, int duration);
		float step(void);

	private:
		float step_;
};

class SinePilot : public Autopilot {

	public:
		SinePilot(float dt, float frequency, float scale);
		~SinePilot(void);

		void set(float start, float stop, int duration);
		float step(void);

	private:
		float frequency_;
		float value_;
		float offset_;
		float amplitude_;
		float scale_;
		float t_;
		float sign_;



};


	}
}

#endif
