/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af.h - Autofocus control algorithm
 */
#pragma once

#include "../af_algorithm.h"
#include "../af_status.h"
#include "../pdaf_data.h"
#include "../pwl.h"

/*
 * This algorithm implements a hybrid of CDAF and PDAF, favouring PDAF.
 *
 * Whenever PDAF is available, it is used in a continuous feedback loop.
 * When triggered in auto mode, we simply enable AF for a limited number
 * frames; the use of both proportional and integral control coefficients
 * should allow it to converge in finite time.
 *
 * When PDAF confidence is low (due e.g. to low contrast or extreme defocus)
 * or PDAF data are absent, fall back to CDAF with a programmed scan pattern.
 * A coarse and fine scan are performed, using ISP's CDAF focus FoM to
 * estimate the lens position with peak contrast. This is slower due to
 * extra latency in the ISP, and requires a settling time between steps.
 *
 * Some hysteresis is applied to the switch between PDAF and CDAF, to avoid
 * "nuisance" scans. During each interval where PDAF is not working, only
 * ONE scan will be performed; CAF cannot track objects using CDAF alone.
 *
 * This algorithm is unrelated to "rpi.focus" which merely reports CDAF FoM.
 */

namespace RPiController {

class Af : public AfAlgorithm
{
public:
	Af(Controller *controller = NULL);
	~Af();
	char const *name() const override;
	void initialise() override;
	int read(const libcamera::YamlObject &params) override
	{
		return cfg_.read(params);
	}

	/* IPA calls */
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

	/* controls */
	void setRange(AfRange range) override;
	void setSpeed(AfSpeed speed) override;
	void setMetering(bool use_windows) override;
	void setWindows(libcamera::Span<libcamera::Rectangle const> const &wins) override;
	bool setLensPosition(double dioptres, int32_t *hwpos) override;
	void enableCAF(bool enable) override;
	void triggerScan() override;
	void cancelScan() override;

private:
	enum class ScanState {
		Idle,
		Pdaf,
		Coarse,
		Fine,
		Settle
	};

	struct CfgParams {
		double focusMin[AfRangeMax];		/* lower (far) limit in dipotres */
		double focusMax[AfRangeMax];		/* upper (near) limit in dioptres */
		double focusDefault[AfRangeMax];	/* default setting ("hyperfocal") */
		double stepCoarse[AfSpeedMax];		/* used for scans */
		double stepFine[AfSpeedMax];		/* used for scans */
		double contrastRatio[AfSpeedMax];	/* used for scan termination and reporting */
		double pdafGain[AfSpeedMax];		/* coefficient for PDAF feedback loop */
		double pdafSquelch[AfSpeedMax];		/* PDAF stability parameter (device-specific) */
		double maxSlew[AfSpeedMax];		/* limit for lens movement per frame */
		uint32_t pdafFrames[AfSpeedMax];	/* number of iterations when triggered */
		uint32_t dropoutFrames[AfSpeedMax];	/* number of non-PDAF frames to switch to CDAF */
		uint32_t stepFrames[AfSpeedMax];	/* frames to skip in between steps of a scan */
		uint32_t confEpsilon;			/* PDAF hysteresis threshold (sensor-specific) */
		uint32_t confThresh;			/* PDAF confidence cell min (sensor-specific) */
		uint32_t confClip;			/* PDAF confidence cell max (sensor-specific) */
		uint32_t skipFrames;			/* frames to skip at start or modeswitch */
		Pwl map;				/* converts dioptres -> lens driver position */

		CfgParams();
		bool readRange(int index, const libcamera::YamlObject &params, char const *name);
		bool readSpeed(int index, const libcamera::YamlObject &params, char const *name);
		int read(const libcamera::YamlObject &params);
	};

	struct ScanRecord {
		double focus;
		double contrast;
		double phase;
		double conf;
	};

	bool getPhase(PdafData const &data, double &phase, double &conf) const;
	double getContrast(struct bcm2835_isp_stats_focus const focus_stats[FOCUS_REGIONS]) const;
	void doPDAF(double phase, double conf);
	bool earlyTerminationByPhase(double phase);
	void doScan(double contrast, double phase, double conf);
	double findPeak(unsigned index) const;
	void doAF(double contrast, double phase, double conf);
	void updateLensPosition();
	void startProgrammedScan();

	/* Configuration and settings */
	CfgParams cfg_;
	AfRange range_;
	AfSpeed speed_;
	libcamera::Size sensorSize_;
	bool cafEnabled_;
	bool useWeights_;
	uint8_t phaseWeights_[PDAF_DATA_ROWS][PDAF_DATA_COLS];
	uint16_t contrastWeights_[FOCUS_REGIONS];

	/* Working state. */
	ScanState scanState_;
	double ftarget_, fsmooth_;
	double prevContrast_;
	bool pdafSeen_;
	unsigned skipCount_, stepCount_, dropCount_;
	unsigned scanMaxIndex_;
	double scanMaxContrast_, scanMinContrast_;
	std::vector<ScanRecord> scanData_;
	AfState reportState_;
};

} // namespace RPiController
