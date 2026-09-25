/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast autofocus
 *
 * The software ISP measures the green contrast in the central ninth of the
 * frame (SwIspStats::sharpness). The lens is swept over its range in coarse
 * steps, stopping once past the peak, then around the best position in fine
 * steps, and parked on the sharpest one. A lasting change of the contrast
 * or of the brightness (the scene has changed) followed by a steady scene
 * starts a local search around the current position, see startClimb().
 */

#include "af.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAf)

namespace ipa::soft::algorithms {

/*
 * Frames to skip after a lens move: the contrast of the next frame is
 * partly changed, the one after has settled (DW9807 in SAC3 mode).
 */
static constexpr unsigned int kSettleFrames = 2;
/* Frames before the first sweep, to let the exposure converge */
static constexpr unsigned int kStartupFrames = 12;
static constexpr int32_t kCoarseSteps = 11;
static constexpr int32_t kFineSteps = 4;
/* A sweep stops after two positions below this fraction of the best */
static constexpr double kFallRatio = 0.7;
/*
 * A change of the contrast or of the brightness of the focus window larger
 * than this, lasting kChangedFrames, is a scene change. The brightness
 * catches the common case of pointing at another subject whose contrast,
 * out of focus, is about the same.
 */
static constexpr double kChangeRatio = 0.25;
static constexpr unsigned int kChangedFrames = 15;
/* The scene is steady when both vary less than this, frame to frame */
static constexpr double kSteadyRatio = 0.08;
static constexpr unsigned int kSteadyFrames = 10;
/*
 * A coarse sweep whose best contrast is not this much above its worst saw
 * no peak (a dark or featureless scene, where noise sets the contrast): its
 * best position is noise. Park the lens mid-range instead.
 */
static constexpr double kPeakRatio = 1.15;
/* A climb step must improve the contrast by this much to count */
static constexpr double kClimbGain = 1.03;

int Af::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	lensMin_ = tuningData["lensMin"].get<int32_t>(-1);
	lensMax_ = tuningData["lensMax"].get<int32_t>(-1);
	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* Sweep only the useful range: the driver's is the whole DAC. */
	AfConfiguration &afConfig = context.configuration.af;
	if (afConfig.enabled && lensMin_ >= 0 && lensMax_ > lensMin_) {
		afConfig.min = std::max(afConfig.min, lensMin_);
		afConfig.max = std::min(afConfig.max, lensMax_);
	}

	state_ = State::Waiting;
	settle_ = kStartupFrames;
	position_ = -1;
	changed_ = 0;
	steady_ = 0;
	last_ = 0.0;
	lastBrightness_ = 0.0;
	referenceBrightness_ = 0.0;
	reference_ = 0.0;

	context.activeState.af.changed = false;

	if (context.configuration.af.enabled)
		LOG(IPASoftAf, Debug)
			<< "Lens range " << context.configuration.af.min
			<< "-" << context.configuration.af.max;

	return 0;
}

void Af::moveTo(IPAContext &context, int32_t position)
{
	position = std::clamp(position, context.configuration.af.min,
			      context.configuration.af.max);
	if (position == position_)
		return;

	position_ = position;
	context.activeState.af.position = position;
	context.activeState.af.changed = true;
	settle_ = kSettleFrames;
}

void Af::startSweep(IPAContext &context, State state, int32_t from,
		    int32_t to, int32_t step)
{
	state_ = state;
	step_ = std::max(step, 1);
	end_ = to;
	best_ = 0.0;
	worst_ = 0.0;
	bestPosition_ = from;
	falling_ = 0;

	/* Measure the start position too, even if the lens is already there. */
	position_ = -1;
	moveTo(context, from);
}

void Af::sweep(IPAContext &context, double sharpness)
{
	if (!worst_ || sharpness < worst_)
		worst_ = sharpness;

	if (sharpness > best_) {
		best_ = sharpness;
		bestPosition_ = position_;
		falling_ = 0;
	} else if (sharpness < best_ * kFallRatio) {
		falling_++;
	}

	if (position_ < end_ && falling_ < 2) {
		moveTo(context, std::min(position_ + step_, end_));
		return;
	}

	const AfConfiguration &af = context.configuration.af;

	if (state_ == State::Coarse && best_ < worst_ * kPeakRatio) {
		LOG(IPASoftAf, Debug)
			<< "No peak (contrast " << worst_ << "-" << best_
			<< "), parking mid-range";
		bestPosition_ = (af.min + af.max) / 2;
		finish(context);
		return;
	}

	if (state_ == State::Coarse) {
		int32_t span = (af.max - af.min) / kCoarseSteps;
		startSweep(context, State::Fine, bestPosition_ - span,
			   std::min(bestPosition_ + span, af.max),
			   span / kFineSteps);
		return;
	}

	LOG(IPASoftAf, Debug)
		<< "Focused at " << bestPosition_ << ", contrast " << best_;

	finish(context);
}

/*
 * After a scene change, search around the current position rather than
 * sweeping the whole range: probe one step on each side, climb towards the
 * sharper one while the contrast grows, and stay put if neither side is
 * better. The lens moves little when the subject distance has not changed,
 * which matters for video.
 */
void Af::startClimb(IPAContext &context, double sharpness)
{
	const AfConfiguration &af = context.configuration.af;

	state_ = State::Climb;
	climbStart_ = position_;
	climbDirection_ = 1;
	step_ = std::max((af.max - af.min) / kCoarseSteps / 2, 1);
	best_ = sharpness;
	bestPosition_ = position_;

	if (position_ + step_ > af.max)
		climbDirection_ = -1;
	moveTo(context, position_ + climbDirection_ * step_);
}

void Af::climb(IPAContext &context, double sharpness)
{
	const AfConfiguration &af = context.configuration.af;

	if (sharpness > best_ * kClimbGain) {
		best_ = sharpness;
		bestPosition_ = position_;

		int32_t next = position_ + climbDirection_ * step_;
		if (next >= af.min && next <= af.max) {
			moveTo(context, next);
			return;
		}
	} else if (bestPosition_ == climbStart_ && climbDirection_ == 1 &&
		   climbStart_ - step_ >= af.min) {
		/* The first probe did not help: try the other side. */
		climbDirection_ = -1;
		moveTo(context, climbStart_ - step_);
		return;
	}

	if (bestPosition_ == climbStart_) {
		LOG(IPASoftAf, Debug) << "Still focused at " << climbStart_;
		finish(context);
		return;
	}

	/* Refine around the best step. */
	startSweep(context, State::Fine, bestPosition_ - step_,
		   std::min(bestPosition_ + step_, af.max),
		   std::max(step_ / 3, 1));
}

void Af::finish(IPAContext &context)
{
	state_ = State::Focused;
	reference_ = best_;
	last_ = best_;
	changed_ = 0;
	steady_ = 0;
	/* Taken from the first frame at the final position, see watch(). */
	referenceBrightness_ = 0.0;
	moveTo(context, bestPosition_);
}

void Af::watch(IPAContext &context, double sharpness, double brightness)
{
	bool different = std::abs(sharpness - reference_) > reference_ * kChangeRatio ||
			 std::abs(brightness - referenceBrightness_) >
				 referenceBrightness_ * kChangeRatio;
	if (different)
		changed_++;
	else if (changed_ < kChangedFrames)
		changed_ = 0;

	if (std::abs(sharpness - last_) < last_ * kSteadyRatio &&
	    std::abs(brightness - lastBrightness_) < lastBrightness_ * kSteadyRatio)
		steady_++;
	else
		steady_ = 0;
	last_ = sharpness;
	lastBrightness_ = brightness;

	if (changed_ >= kChangedFrames && steady_ >= kSteadyFrames) {
		LOG(IPASoftAf, Debug)
			<< "Scene changed, contrast " << reference_
			<< " -> " << sharpness;
		startClimb(context, sharpness);
		return;
	}

	/* Follow slow drifts (exposure, noise) of an unchanged scene. */
	if (!changed_) {
		reference_ = 0.9 * reference_ + 0.1 * sharpness;
		referenceBrightness_ = 0.9 * referenceBrightness_ + 0.1 * brightness;
	}
}

void Af::process(IPAContext &context,
		 const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	if (!context.configuration.af.enabled)
		return;

	LOG(IPASoftAf, Debug)
		<< "frame " << frame << " position " << position_
		<< " settle " << settle_ << " contrast "
		<< (stats->sharpnessBase ? static_cast<double>(stats->sharpness) /
						   stats->sharpnessBase
					 : 0.0)
		<< " base " << stats->sharpnessBase;

	if (settle_) {
		settle_--;
		return;
	}

	if (!stats->sharpnessBase)
		return;

	/* Contrast relative to the brightness, independent of the exposure */
	double sharpness = static_cast<double>(stats->sharpness) /
			   stats->sharpnessBase;
	double brightness = static_cast<double>(stats->sharpnessBase);

	switch (state_) {
	case State::Waiting: {
		const AfConfiguration &af = context.configuration.af;
		startSweep(context, State::Coarse, af.min, af.max,
			   (af.max - af.min) / kCoarseSteps);
		break;
	}
	case State::Coarse:
	case State::Fine:
		sweep(context, sharpness);
		break;
	case State::Climb:
		climb(context, sharpness);
		break;
	case State::Focused:
		/* The references come from the first frame at the final position. */
		if (!referenceBrightness_) {
			referenceBrightness_ = brightness;
			lastBrightness_ = brightness;
			reference_ = sharpness;
			last_ = sharpness;
		}
		watch(context, sharpness, brightness);
		break;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
