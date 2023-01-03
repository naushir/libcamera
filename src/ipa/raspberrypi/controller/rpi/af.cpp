
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af.cpp - PDAF control algorithm
 */

#include "af.h"

#include <iomanip>
#include <math.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiAf)

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

#define NAME "rpi.af"

/*
 * Default values for parameters. All may be overridden in the tuning file.
 * Many of these values are sensor- or module-dependent; the defaults here
 * assume IMX708 in a Raspberry Pi V3 camera with the standard lens.
 *
 * Here all focus values are in dioptres (1/m). They are converted to hardware
 * units when written to status.focusSetting or returned from setLensPosition.
 *
 * Gain and delay values are relative to frame interval, since much (not all)
 * of the delay is in the sensor and (for CDAF) ISP, not the lens mechanism.
 */

namespace {

constexpr double DefaultFocusMin = 0.0; 	/* infinity */
constexpr double DefaultFocusMax = 12.0;	/* closest focus, about 83 mm */
constexpr double DefaultFocusDefault = 1.0;	/* about 1 m, guessed "hyperfocal" */
constexpr uint32_t DefaultConfEpsilon = 8;	/* PDAF confidence hysteresis level */
constexpr uint32_t DefaultConfThreshold = 16;	/* PDAF cell confidence threshold */
constexpr uint32_t DefaultConfClip = 512;	/* PDAF cell confidence limit */
constexpr double DefaultGain = -0.02;		/* loop gain, should be small and -ve */
constexpr double DefaultSquelch = 0.125;	/* suppress small changes of lens pos */
constexpr uint32_t DefaultSkipFrames = 5;	/* initial frames to skip */
constexpr uint32_t DefaultStepFrames = 4;	/* skip frames per step during scan */
constexpr double DefaultStepCoarse = 1.0;	/* step size for coarse scan */
constexpr double DefaultStepFine = 0.25;	/* step size for fine scan */
constexpr uint32_t DefaultMaxSlew = 2.0;	/* largest lens change per frame */
constexpr uint32_t DefaultPdafFrames = 16;	/* Number of iterations in auto mode */
constexpr uint32_t DefaultDropoutFrames = 6;	/* Absence of PDAF to switch to CDAF */
constexpr double DefaultContrastRatio = 0.75;	/* min/max contrast ratio for success */
constexpr double DefaultMapX0 = -1.0;		/* slightly beyond infinity */
constexpr double DefaultMapY0 = 420.0;		/* corresponding lens setting */
constexpr double DefaultMapX1 = 15.0;		/* close to physical end stop */
constexpr double DefaultMapY1 = 960;		/* corresponding lens setting */

}

Af::CfgParams::CfgParams()
	: focusMin{ DefaultFocusMin, DefaultFocusMin, DefaultFocusMin },
	  focusMax{ DefaultFocusMax, DefaultFocusMax, DefaultFocusMax },
	  focusDefault{ DefaultFocusDefault, DefaultFocusDefault, DefaultFocusDefault },
	  stepCoarse{ DefaultStepCoarse, DefaultStepCoarse },
	  stepFine{ DefaultStepFine, DefaultStepFine },
	  contrastRatio{ DefaultContrastRatio, DefaultContrastRatio },
	  pdafGain{ DefaultGain, DefaultGain },
	  pdafSquelch{ DefaultSquelch, DefaultSquelch },
	  maxSlew{ DefaultMaxSlew, DefaultMaxSlew },
	  pdafFrames{ DefaultPdafFrames, DefaultPdafFrames },
	  dropoutFrames{ DefaultDropoutFrames, DefaultDropoutFrames },
	  stepFrames{ DefaultStepFrames, DefaultStepFrames },
	  confEpsilon(DefaultConfEpsilon),
	  confThresh(DefaultConfThreshold),
	  confClip(DefaultConfClip),
	  skipFrames(DefaultSkipFrames),
	  map()
{
}

template<typename T>
static void readNumber(T &dest, const libcamera::YamlObject &params, char const *name)
{
	auto value = params[name].get<T>();
	if (!value) {
		LOG(RPiAf, Warning) << "Missing parameter \"" << name << "\"";
	} else {
		dest = *value;
	}
}

bool Af::CfgParams::readRange(int index, const libcamera::YamlObject &ranges, char const *name)
{
	if (ranges.contains(name)) {
		const libcamera::YamlObject &r = ranges[name];

		readNumber<double>(focusMin[index], r, "min");
		readNumber<double>(focusMax[index], r, "max");
		readNumber<double>(focusDefault[index], r, "default");
		return true;
	}

	return false;
}

bool Af::CfgParams::readSpeed(int index, const libcamera::YamlObject &speeds, char const *name)
{
	if (speeds.contains(name)) {
		const libcamera::YamlObject &s = speeds[name];

		readNumber<double>(stepCoarse[index], s, "step_coarse");
		readNumber<double>(stepFine[index], s, "step_fine");
		readNumber<double>(contrastRatio[index], s, "contrast_ratio");
		readNumber<double>(pdafGain[index], s, "pdaf_gain");
		readNumber<double>(pdafSquelch[index], s, "pdaf_squelch");
		readNumber<double>(maxSlew[index], s, "max_slew");
		readNumber<uint32_t>(pdafFrames[index], s, "pdaf_frames");
		readNumber<uint32_t>(dropoutFrames[index], s, "dropout_frames");
		readNumber<uint32_t>(stepFrames[index], s, "step_frames");
		return true;
	}

	return false;
}

int Af::CfgParams::read(const libcamera::YamlObject &params)
{
	if (params.contains("ranges")) {
		auto &ranges = params["ranges"];

		if (!readRange(0, ranges, "normal"))
			LOG(RPiAf, Warning) << "Missing range \"normal\"";
		focusMin[1] = focusMin[0];
		focusMax[1] = focusMax[0];
		focusDefault[1] = focusDefault[0];
		readRange(1, ranges, "macro");
		focusMin[2] = std::min(focusMin[0], focusMin[1]);
		focusMax[2] = std::max(focusMax[0], focusMax[1]);
		focusDefault[2] = focusDefault[0];
		readRange(2, ranges, "full");
	} else {
		LOG(RPiAf, Warning) << "No ranges defined";
	}

	if (params.contains("speeds")) {
		auto &speeds = params["speeds"];

		if (!readSpeed(0, speeds, "normal"))
			LOG(RPiAf, Warning) << "Missing speed \"normal\"";
		stepCoarse[1] = stepCoarse[0];
		stepFine[1] = stepFine[0];
		contrastRatio[1] = contrastRatio[0];
		pdafGain[1] = pdafGain[0];
		pdafSquelch[1] = pdafSquelch[0];
		maxSlew[1] = maxSlew[0];
		pdafFrames[1] = pdafFrames[0];
		dropoutFrames[1] = dropoutFrames[0];
		stepFrames[1] = stepFrames[0];
		readSpeed(1, speeds, "fast");
	} else {
		LOG(RPiAf, Warning) << "No speeds defined";
	}

	readNumber<uint32_t>(confEpsilon, params, "conf_epsilon");
	readNumber<uint32_t>(confThresh, params, "conf_thresh");
	readNumber<uint32_t>(confClip, params, "conf_clip");
	readNumber<uint32_t>(skipFrames, params, "skip_frames");

	if (params.contains("map"))
		map.read(params["map"]);
	else
		LOG(RPiAf, Warning) << "No map defined";

	return 0;
}

/* Af Algorithm class */

Af::Af(Controller *controller)
	: AfAlgorithm(controller),
	  cfg_(),
	  range_(AfRangeNormal),
	  speed_(AfSpeedNormal),
	  sensorSize_{ 4608, 2592 },
	  cafEnabled_(false),
	  useWeights_(false),
	  phaseWeights_{},
	  contrastWeights_{},
	  scanState_(ScanState::Idle),
	  ftarget_(-INFINITY),
	  fsmooth_(-INFINITY),
	  prevContrast_(0.0),
	  skipCount_(DefaultSkipFrames),
	  stepCount_(0),
	  dropCount_(0),
	  scanMaxContrast_(0.0),
	  scanMinContrast_(1.0e9),
	  scanData_(),
	  reportState_(AfState::Unknown)
{
	scanData_.reserve(24);
}

Af::~Af()
{
}

char const *Af::name() const
{
	return NAME;
}

void Af::initialise()
{
	if (cfg_.map.empty()) {
		cfg_.map.append(DefaultMapX0, DefaultMapY0);
		cfg_.map.append(DefaultMapX1, DefaultMapY1);
	}
}

void Af::switchMode(CameraMode const &cameraMode, Metadata *metadata)
{
	(void)metadata;
	sensorSize_.width = cameraMode.sensorWidth;
	sensorSize_.height = cameraMode.sensorHeight;

	if (scanState_ >= ScanState::Coarse && scanState_ < ScanState::Settle) {
		/* If a scan was in progress, re-start it, as CDAF statistics
		 * may have changed. Though if the application is just about
		 * to take a still picture, this will not help...
		 */
		startProgrammedScan();
	}
	skipCount_ = cfg_.skipFrames;
}

bool Af::getPhase(PdafData const &data, double &phase, double &conf) const
{
	static const uint8_t DEFAULT_WEIGHTS[PDAF_DATA_ROWS][PDAF_DATA_COLS] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
	};
	int32_t sum_w = 0;
	int32_t sum_wc = 0;
	int32_t sum_wcp = 0;
	auto ww = useWeights_ ? phaseWeights_ : DEFAULT_WEIGHTS;

	for (unsigned i = 0; i < PDAF_DATA_ROWS; ++i) {
		for (unsigned j = 0; j < PDAF_DATA_COLS; ++j) {
			if (ww[i][j]) {
				uint32_t c = data.conf[i][j];
				if (c >= cfg_.confThresh) {
					if (c > cfg_.confClip)
						c = cfg_.confClip;
					sum_wc += ww[i][j] * c;
					c -= (cfg_.confThresh >> 1);
					sum_wcp += ww[i][j] * data.phase[i][j] * (int32_t)c;
				}
				sum_w += ww[i][j];
			}
		}
	}

	if (sum_wc > 0) {
		phase = (double)sum_wcp / (double)sum_wc;
		conf = (double)sum_wc / (double)sum_w;
		return true;
	} else {
		phase = 0.0;
		conf = 0.0;
		return false;
	}
}

void Af::doPDAF(double phase, double conf)
{
	/* Apply gain; scale down when confidence is low or error is small */
	phase *= cfg_.pdafGain[speed_];
	phase *= conf / (conf + cfg_.confEpsilon);
	if (std::abs(phase) < cfg_.pdafSquelch[speed_]) {
		if (scanState_ == ScanState::Pdaf && stepCount_ + 3 < cfg_.pdafFrames[speed_])
			scanState_ = ScanState::Idle;
		else
			phase *= std::pow(std::abs(phase) / cfg_.pdafSquelch[speed_], 2.0);
	}

	/* Apply slew rate limit. Report failure if out of bounds. */
	if (phase < -cfg_.maxSlew[speed_]) {
		phase = -cfg_.maxSlew[speed_];
		reportState_ = (ftarget_ <= cfg_.focusMin[range_]) ? AfState::Failed : AfState::Scanning;
	} else if (phase > cfg_.maxSlew[speed_]) {
		phase = cfg_.maxSlew[speed_];
		reportState_ = (ftarget_ >= cfg_.focusMax[range_]) ? AfState::Failed : AfState::Scanning;
	} else if (scanState_ == ScanState::Idle) {
		reportState_ = AfState::Focused;
	}

	ftarget_ = fsmooth_ + phase;
}

bool Af::earlyTerminationByPhase(double phase)
{
	if (scanData_.size() > 0 &&
	    scanData_[scanData_.size() - 1].conf >= cfg_.confEpsilon) {
		double oldFocus = scanData_[scanData_.size() - 1].focus;
		double oldPhase = scanData_[scanData_.size() - 1].phase;

		/*
		 * Check that the gradient is finite and has the expected sign;
		 * Interpolate/extrapolate the lens position for zero phase.
		 * Check that the extrapolation is well-conditioned.
		 */
		if ((ftarget_ - oldFocus) * (phase - oldPhase) >= 0.01) {
			double param = phase / (phase - oldPhase);
			if (-3.0 <= param && param <= 3.5) {
				ftarget_ += param * (oldFocus - ftarget_);
				LOG(RPiAf, Debug) << "ETBP: param=" << param;
				return true;
			}
		}
	}

	return false;
}

double Af::findPeak(unsigned i) const
{
	double f = scanData_[i].focus;

	if (i > 0 && i + 1 < scanData_.size()) {
		double dropLo = scanData_[i].contrast - scanData_[i - 1].contrast;
		double dropJi = scanData_[i].contrast - scanData_[i + 1].contrast;
		if (dropLo < dropJi) {
			double param = 0.3125 * (1.0 - dropLo / dropJi) * (1.6 - dropLo / dropJi);
			f += param * (scanData_[i - 1].focus - f);
		} else if (dropJi < dropLo) {
			double param = 0.3125 * (1.0 - dropJi / dropLo) * (1.6 - dropJi / dropLo);
			f += param * (scanData_[i + 1].focus - f);
		}
	}

	LOG(RPiAf, Debug) << "FindPeak: " << f;
	return f;
}

void Af::doScan(double contrast, double phase, double conf)
{
	/* Record lens position, contrast and phase values for the current scan */
	if (scanData_.empty() || contrast > scanMaxContrast_) {
		scanMaxContrast_ = contrast;
		scanMaxIndex_ = scanData_.size();
	}
	if (contrast < scanMinContrast_)
		scanMinContrast_ = contrast;
	scanData_.push_back(ScanRecord{ ftarget_, contrast, phase, conf });

	if (scanState_ == ScanState::Coarse) {
		if (ftarget_ >= cfg_.focusMax[range_] ||
		    contrast < cfg_.contrastRatio[speed_] * scanMaxContrast_) {
			/*
			 * Finished course scan, or termination based on contrast.
			 * Jump to just after max contrast and start fine scan.
			 */
			ftarget_ = std::min(ftarget_, findPeak(scanMaxIndex_) + 2.0 * cfg_.stepFine[speed_]);
			scanState_ = ScanState::Fine;
			scanData_.clear();
		} else {
			ftarget_ += cfg_.stepCoarse[speed_];
		}
	} else { /* ScanState::Fine */
		if (ftarget_ <= cfg_.focusMin[range_] || scanData_.size() >= 5 ||
		    contrast < cfg_.contrastRatio[speed_] * scanMaxContrast_) {
			/*
			 * Finished fine scan, or termination based on contrast.
			 * Use quadratic peak-finding to find best contrast position.
			 */
			ftarget_ = findPeak(scanMaxIndex_);
			scanState_ = ScanState::Settle;
		} else {
			ftarget_ -= cfg_.stepFine[speed_];
		}
	}

	stepCount_ = (ftarget_ == fsmooth_) ? 0 : cfg_.stepFrames[speed_];
}

void Af::doAF(double contrast, double phase, double conf)
{
	/* Skip frames at startup and after mode change */
	if (skipCount_ > 0) {
		LOG(RPiAf, Debug) << "SKIP";
		skipCount_--;
		return;
	}

	if (scanState_ == ScanState::Pdaf ||
	    (cafEnabled_ && scanState_ == ScanState::Idle && cfg_.dropoutFrames[speed_] > 0)) {
		/*
		 * Use PDAF closed-loop control whenever available, in both CAF
		 * mode and (for a limited number of iterations) when triggered.
		 * If PDAF fails (due to poor contrast, noise or large defocus),
		 * fall back to a CDAF-based scan. To avoid "nuisance" scans,
		 * scan only after a number of frames with low PDAF confidence.
		 */
		if (conf > (dropCount_ ? cfg_.confEpsilon : 0.0)) {
			if (stepCount_ > 0)
				stepCount_--;
			else
				scanState_ = ScanState::Idle;
			doPDAF(phase, conf);
			dropCount_ = 0;
		} else if (++dropCount_ == cfg_.dropoutFrames[speed_]) {
			startProgrammedScan();
		}
	}

	else if (scanState_ >= ScanState::Coarse && fsmooth_ == ftarget_) {
		/*
		 * Scanning sequence. This means PDAF has become unavailable.
		 * Allow a delay between steps for CDAF FoM statistics to be
		 * updated, and a "settling time" at the end of the sequence.
		 * [A coarse or fine scan can be abandoned if two PDAF samples
		 * allow direct interpolation of the zero-phase lens position.]
		 */
		if (stepCount_ > 0) {
			stepCount_--;
		} else if (scanState_ == ScanState::Settle) {
			if (prevContrast_ >= cfg_.contrastRatio[speed_] * scanMaxContrast_ &&
			    scanMinContrast_ <= cfg_.contrastRatio[speed_] * scanMaxContrast_)
				reportState_ = AfState::Focused;
			else
				reportState_ = AfState::Failed;
			scanState_ = ScanState::Idle;
			scanData_.clear();
		} else if (conf >= cfg_.confEpsilon && earlyTerminationByPhase(phase)) {
			scanState_ = ScanState::Settle;
			stepCount_ = cafEnabled_ ? 0 : cfg_.stepFrames[speed_];
		} else {
			doScan(contrast, phase, conf);
		}
	}
}

void Af::updateLensPosition()
{
	/* clamp ftarget_ to range limits, and fsmooth_ to slew rate limit */
	if (isfinite(ftarget_)) {
		ftarget_ = std::clamp(ftarget_,
				      cfg_.focusMin[range_],
				      cfg_.focusMax[range_]);
		if (isfinite(fsmooth_)) {
			fsmooth_ = std::clamp(ftarget_,
					      fsmooth_ - cfg_.maxSlew[speed_],
					      fsmooth_ + cfg_.maxSlew[speed_]);
		} else {
			fsmooth_ = ftarget_;
			skipCount_ = cfg_.skipFrames;
		}
	}
}

/*
 * PDAF phase data are available in prepare(), but CDAF statistics are not
 * available until process(). We are gambling on the availability of PDAF.
 * To expedite feedback control using PDAF, issue the V4L2 lens control from
 * prepare(). Conversely, during scans, we must allow an extra frame delay
 * between steps, to retrieve CDAF statistics from the previous process()
 * so we can terminate the scan early without having to change our minds.
 */

void Af::prepare(Metadata *imageMetadata)
{
	/* Remember the previous lens position before we change it */
	AfStatus status;
	status.lensKnown = isfinite(fsmooth_);
	status.lensEstimate = isfinite(fsmooth_) ? fsmooth_ : 0.0;

	/* Get PDAF from the embedded metadata, and run AF algorithm core */
	PdafData data;
	double phase = 0.0, conf = 0.0;
	double oldFt = ftarget_;
	double oldFs = fsmooth_;
	ScanState oldSs = scanState_;
	uint32_t oldSt = stepCount_;
	if (imageMetadata->get("pdaf.data", data) == 0)
		getPhase(data, phase, conf);
	doAF(prevContrast_, phase, conf);
	updateLensPosition();
	LOG(RPiAf, Debug) << std::fixed << std::setprecision(2)
			  << static_cast<unsigned int>(reportState_)
			  << " sst" << static_cast<unsigned int>(oldSs) << "->" << static_cast<unsigned int>(scanState_)
			  << " stp" << oldSt << "->" << stepCount_
			  << " ft" << oldFt << "->" << ftarget_
			  << " fs" << oldFs << "->" << fsmooth_
			  << " cont=" << (int)prevContrast_ << " phase=" << (int)phase << " conf=" << (int)conf;

	/* Report status and produce new lens setting */
	status.state = reportState_;
	status.lensDriven = isfinite(fsmooth_);
	status.lensSetting = isfinite(fsmooth_) ? cfg_.map.eval(fsmooth_) : 0.0;
	imageMetadata->set("af.status", status);
}

double Af::getContrast(struct bcm2835_isp_stats_focus const focus_stats[FOCUS_REGIONS]) const
{
	uint32_t tot_w = 0, tot_wc = 0;

	if (useWeights_) {
		for (unsigned i = 0; i < FOCUS_REGIONS; ++i) {
			unsigned w = contrastWeights_[i];
			tot_w += w;
			tot_wc += w * (focus_stats[i].contrast_val[1][1] >> 10);
		}
	}
	if (tot_w == 0) {
		tot_w = 2;
		tot_wc = (focus_stats[5].contrast_val[1][1] >> 10) +
			 (focus_stats[6].contrast_val[1][1] >> 10);
	}

	return (double)tot_wc / (double)tot_w;
}

void Af::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	(void)imageMetadata;
	prevContrast_ = getContrast(stats->focus_stats);
}

/* Controls */

void Af::setRange(AfRange r)
{
	LOG(RPiAf, Debug) << "setRange: " << (unsigned)r;
	if ((unsigned)r < AfAlgorithm::AfRangeMax)
		range_ = r;
}

void Af::setSpeed(AfSpeed s)
{
	LOG(RPiAf, Debug) << "setSpeed: " << (unsigned)s;
	if ((unsigned)s < AfAlgorithm::AfSpeedMax) {
		if (scanState_ == ScanState::Pdaf && cfg_.pdafFrames[s] > cfg_.pdafFrames[speed_])
			stepCount_ += cfg_.pdafFrames[s] - cfg_.pdafFrames[speed_];
		speed_ = s;
	}
}

void Af::setMetering(bool mode)
{
	useWeights_ = mode;
}

void Af::setWindows(libcamera::Span<libcamera::Rectangle const> const &wins)
{
	/*
	 * Here we just merge all of the given windows, weighted by area.
	 * If there are more than 15 overlapping windows, overflow can occur.
	 * TODO: A better approach might be to find the phase in each window
	 * and choose either the closest or the highest-confidence one?
	 *
	 * Using mostly "int" arithmetic, because Rectangle has signed x, y
	 */
	int gridY = (int)(sensorSize_.height / PDAF_DATA_ROWS);
	int gridX = (int)(sensorSize_.width / PDAF_DATA_COLS);
	int gridA = gridY * gridX;

	for (int i = 0; i < PDAF_DATA_ROWS; ++i)
		std::fill(phaseWeights_[i], phaseWeights_[i] + PDAF_DATA_COLS, 0);
	std::fill(contrastWeights_, contrastWeights_ + FOCUS_REGIONS, 0);

	for (auto &w : wins) {
		for (int i = 0; i < PDAF_DATA_ROWS; ++i) {
			int y0 = std::max(gridY * i, w.y);
			int y1 = std::min(gridY * (i + 1), w.y + (int)(w.height));
			if (y0 >= y1)
				continue;
			y1 -= y0;
			for (int j = 0; j < PDAF_DATA_COLS; ++j) {
				int x0 = std::max(gridX * j, w.x);
				int x1 = std::min(gridX * (j + 1), w.x + (int)(w.width));
				if (x0 >= x1)
					continue;
				int a = y1 * (x1 - x0);
				a = (16 * a + gridA - 1) / gridA;
				phaseWeights_[i][j] += a;
				contrastWeights_[4 * ((3 * i) / PDAF_DATA_ROWS) + ((4 * j) / PDAF_DATA_COLS)] += a;
			}
		}
	}
}

bool Af::setLensPosition(double dioptres, int *hwpos)
{
	bool changed = false;

	LOG(RPiAf, Debug) << "setLensPosition: " << dioptres;
	if (reportState_ != AfState::Scanning) {
		changed = !(fsmooth_ == dioptres);
		ftarget_ = dioptres;
		updateLensPosition();
	}

	if (hwpos)
		*hwpos = cfg_.map.eval(fsmooth_);

	return changed;
}

void Af::enableCAF(bool enabled)
{
	LOG(RPiAf, Debug) << "enableCAF: " << enabled;
	if (enabled && !cafEnabled_) {
		if (cfg_.dropoutFrames[speed_] > 0) {
			if (!isfinite(ftarget_)) {
				ftarget_ = cfg_.focusDefault[range_];
				updateLensPosition();
			}
			scanState_ = ScanState::Idle;
			scanData_.clear();
			dropCount_ = 0;
			reportState_ = AfState::Scanning;
		} else {
			startProgrammedScan();
		}
	}
	cafEnabled_ = enabled;
}

void Af::cancelScan()
{
	LOG(RPiAf, Debug) << "cancelScan";
	scanState_ = ScanState::Idle;
	scanData_.clear();
	reportState_ = AfState::Unknown;
}

void Af::startProgrammedScan()
{
	ftarget_ = cfg_.focusMin[range_];
	updateLensPosition();
	scanState_ = ScanState::Coarse;
	scanMaxContrast_ = 0.0;
	scanMinContrast_ = 1.0e9;
	scanMaxIndex_ = 0;
	scanData_.clear();
	stepCount_ = cfg_.stepFrames[speed_];
	reportState_ = AfState::Scanning;
}

void Af::triggerScan()
{
	LOG(RPiAf, Debug) << "triggerScan";
	if (scanState_ == ScanState::Idle) {
		if (cfg_.pdafFrames[speed_] > 0 && cfg_.dropoutFrames[speed_] > 0) {
			if (!isfinite(ftarget_)) {
				ftarget_ = cfg_.focusDefault[range_];
				updateLensPosition();
			}
			stepCount_ = cfg_.pdafFrames[speed_];
			scanState_ = ScanState::Pdaf;
			dropCount_ = 0;
		} else {
			startProgrammedScan();
		}
		reportState_ = AfState::Scanning;
	}
}

// Register algorithm with the system.
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Af(controller);
}
static RegisterAlgorithm reg(NAME, &create);
