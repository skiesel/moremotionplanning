#pragma once

class BetaDistribution {
public:
	BetaDistribution(double alpha, double beta) : alpha(alpha), beta(beta) {}

	~BetaDistribution() {}

	void incrementAlpha() {
		alpha++;
	}

	void incrementBeta() {
		beta++;
	}

	void decrementAlpha() {
		if(alpha >= 1) {
			alpha--;
		}
	}

	void decrementBeta() {
		if(beta >= 1) {
			beta--;
		}
	}

	double getProbabilityOfSuccess() const {
		return alpha / (alpha + beta);
	}

	double getExpectedTrialsForSuccess() const {
		return 1 / getProbabilityOfSuccess();
	}

	double alpha, beta;
};