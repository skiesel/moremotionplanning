#pragma once

class GaussianDistribution {
public:
	GaussianDistribution() {}

	double getMu() const {
		return mu;
	}

	double getSigma() {
		if(needUpdate) {
			sigma = sqrt(sn / datapoints);
			needUpdate = false;
		}
		return sigma;
	}

	void addDataPoint(double value) {
		datapoints++;
		double oldMu = mu;
		mu += (value - mu) / datapoints;
		sn += (value - oldMu) * (value - mu);
		needUpdate = true;
	}

	void removeDataPoint(double value) {
		datapoints--;
		double oldMu = mu;
		mu -= (value - mu) / datapoints;
		sn -= (value - oldMu) * (value - mu);
		needUpdate = true;
	}

	double getCDF(double value) {
		return 0.5 * (1 + std::erf((value - mu) / (sqrt(2 * getSigma()))));
	}

	void add(GaussianDistribution &a, GaussianDistribution &b) {
		mu = a.mu + b.mu;
		sigma = a.getSigma() + b.getSigma();
	}

protected:
	double mu = 0, sn = 0, sigma = 0;
	double datapoints = 0;
	bool needUpdate = false;
};