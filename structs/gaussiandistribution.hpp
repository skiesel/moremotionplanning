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
			assert(!std::isnan(sigma) && !std::isinf(sigma) && !std::isnan(sigma) && !std::isinf(sigma));
		}
		return sigma;
	}

	void addDataPoint(double value) {
		datapoints++;
		assert(value >= 0);
		double oldMu = mu;
		mu += (value - mu) / datapoints;
		sn += (value - oldMu) * (value - mu);
		needUpdate = true;
		assert(!std::isnan(mu) && !std::isinf(mu) && !std::isnan(sn) && !std::isinf(sn));
	}

	void removeDataPoint(double value) {
		assert(value >= 0);
		datapoints--;
		if(datapoints <= 0) {
			mu = 0;
			sn = 0;
			sigma = 0;
			datapoints = 0;
			needUpdate = false;
			return;
		}
		double oldMu = mu;
		mu -= (value - mu) / datapoints;
		sn -= (value - oldMu) * (value - mu);

		if(sn <= 0) {
			sn = 0;
		}
		needUpdate = true;

		assert(!std::isnan(mu) && !std::isinf(mu) && !std::isnan(sn) && !std::isinf(sn));
	}

	double getCDF(double value) {
		return 0.5 * (1 + std::erf((value - mu) / (getSigma() * sqrt(2))));
	}

	GaussianDistribution operator+(GaussianDistribution a) {
		GaussianDistribution gd;
		gd.mu = mu + a.mu;
		gd.sigma = getSigma() + a.getSigma();
		return gd;
	}

	GaussianDistribution operator+(double v) {
		GaussianDistribution gd;
		gd.mu = mu + v;
		gd.sigma = getSigma();
		return gd;
	}

	GaussianDistribution operator-(double v) {
		GaussianDistribution gd;
		gd.mu = mu - v;
		gd.sigma = getSigma();
		return gd;
	}

	GaussianDistribution operator*(double v) {
		GaussianDistribution gd;
		gd.mu = v * mu;
		gd.sigma = getSigma();
		return gd;
	}

protected:
	double mu = 0, sn = 0, sigma = 0;
	double datapoints = 0;
	bool needUpdate = false;
};