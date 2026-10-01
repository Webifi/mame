// license:BSD-3-Clause
// Exact integration of a real pole and two conjugate pole pairs.
// Configured by the board, not by the DOC register or oscillator state.
#ifndef MAME_SOUND_DAC_FILTER_H
#define MAME_SOUND_DAC_FILTER_H

#pragma once

#include <array>
#include <cassert>
#include <complex>
#include <vector>

class dac_filter
{
public:
	using modes = std::array<std::complex<double>, 3>;
	void configure(unsigned clock, unsigned rate, const modes &poles, const modes &gains, unsigned mode_count = 3)
	{
		assert(clock && rate && mode_count && mode_count <= 3);
		m_mode_count = mode_count;
		m_steps.resize((clock + rate - 1) / rate + 1);
		for (unsigned clocks = 0; clocks < m_steps.size(); ++clocks)
			for (unsigned mode = 0; mode < m_mode_count; ++mode)
			{
				const auto e = std::exp(poles[mode] * (double(clocks) / clock));
				const auto j = gains[mode] * (e - 1.0) / poles[mode] * double(clock) * (mode ? 2.0 : 1.0);
				m_steps[clocks][mode] = {e.real(), e.imag(), j.real(), j.imag()};
			}
	}

	double step(double *state, double input, unsigned clocks) const
	{
		assert(clocks < m_steps.size());
		// Each mode obeys w'=p*(w-input), with output g*w. Integrate the
		// exponential exactly over this constant DAC hold. The gains sum
		// to unity; conjugate pairs contribute twice their real part.
		double integral = input * clocks;
		for (unsigned mode = 0; mode < m_mode_count; ++mode)
		{
			const auto &c = m_steps[clocks][mode];
			const double r = state[2 * mode] - input, i = state[2 * mode + 1];
			integral += r * c[2] - i * c[3];
			state[2 * mode] = input + c[0] * r - c[1] * i;
			state[2 * mode + 1] = c[1] * r + c[0] * i;
		}
		return integral;
	}

private:
	unsigned m_mode_count = 0;
	std::vector<std::array<std::array<double, 4>, 3>> m_steps;
};

#endif // MAME_SOUND_DAC_FILTER_H
