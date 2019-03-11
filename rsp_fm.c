/*
* rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
* Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
* Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
* Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
* 
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "rsp_tcp_api.h"

#include <unistd.h>
#include <time.h>
#include <math.h>

#include <mirsdrapi-rsp.h>
#include "rsp_headers.h"

// temporary fixed resolution of 2.048 Mhz / 1 kHz = 2048 IQ samples
#define FFT_INDEX (11)
#define FFT_SIZE (2048)

// buffer sizes: 2048 IQ -> 196 IQ -> (196 mono or 48 IQ) -> 48 mono  -> 480 OUT
#define LEN_FM_W (3 * FFT_SIZE / 32 )
#define LEN_FM_N (LEN_FM_W / 4)
#define LEN_OUT (10 * LEN_FM_N)

// IQ input, 196 sample pairs, output as 48 samples audio
int16_t fm_buff[LEN_FM_W * 2];

// audio output, 48 samples per ms
int16_t	result48[LEN_OUT];


#define MAX(x, y) (((x) > (y)) ? (x) : (y))

int16_t* IQ = NULL;
float *avg = NULL;
int freq;
int rate;
int samples;
FILE *file;

typedef enum {
MODE_FM,
MODE_USB,
MODE_RAW,
MODE_LSB,
MODE_AM
} demod_t;
demod_t mode_demod = MODE_FM;

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len - 1];
	s[len - 1] = '\0';
	switch (last) {
	case 'g':
	case 'G':
		suff *= 1e3;
		/* fall-through */
	case 'm':
	case 'M':
		suff *= 1e3;
		/* fall-through */
	case 'k':
	case 'K':
		suff *= 1e3;
		suff *= atof(s);
		s[len - 1] = last;
		return suff;
	}
	s[len - 1] = last;
	return atof(s);
}

uint32_t get_frequency(char *optarg)
{
	double lower, upper, freq; // max_size is ignored
	char *start, *stop, *step;

	/* hacky string parsing */
	start = strdup(optarg);
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	lower = atofs(start);
	upper = atofs(stop);
	// max_size = atofs(step);
	free(start);

	freq = (lower + upper) / 2.0;
	if (freq < 1.048e6) // reasonable minimum
		freq = 1.048e6;
	if (freq > 2.0e9) // unreasonable maximum
		freq = 2.0e9;
	return (uint32_t)freq;
}

static volatile int do_exit = 0;

#define RSP_FM_VERSION_MAJOR (1)
#define RSP_FM_VERSION_MINOR (0)

#define MAX_DECIMATION_FACTOR (64)
#define MAX_DEVS 4
#define WORKER_TIMEOUT_SEC 3
#define DEFAULT_BW_T mir_sdr_BW_1_536
#define DEFAULT_FREQUENCY (100000000)
#define DEFAULT_SAMPLERATE (2048000)
#define DEFAULT_AGC_SETPOINT -30
#define DEFAULT_GAIN_REDUCTION 40
#define DEFAULT_LNA_STATE 9
#define DEFAULT_AGC_STATE 0
#define RTLSDR_TUNER_R820T 5
#define DEFAULT_GAIN 16;
#define MAX_GAIN (GAIN_STEPS - 1)

static int bwType = DEFAULT_BW_T;
static int infoOverallGr;
static int samples_per_packet;
static int last_gain_idx = 0;
static int verbose = 0;
static int extended_mode = 0;
static int hardware_version = 0;
static rsp_capabilities_t *hardware_caps = NULL;
static rsp_model_t hardware_model = RSP_MODEL_UNKNOWN;
static rsp_tcp_sample_format_t sample_format = RSP_TCP_SAMPLE_FORMAT_UINT8;
static rsp_band_t current_band = BAND_UNKNOWN;
static int current_antenna_input = 0;
static unsigned int current_frequency;
static unsigned int lna_state = DEFAULT_LNA_STATE;
static unsigned int agc_state = DEFAULT_AGC_STATE;
static int agc_set_point = DEFAULT_AGC_SETPOINT;
static int gain_reduction = DEFAULT_GAIN_REDUCTION;
static int am_port = -1;
static uint8_t const_if_gain = 12;
static int sample_shift = 2;

// *************************************

static void sighandler(int signum)
{
	fprintf(stderr, "Signal (%d) caught: Exit!\n", signum);
	do_exit = 1;
}

void gc_callback(unsigned int gRdB, unsigned int lnaGRdB, void* cbContext)
{
	if (gRdB == mir_sdr_ADC_OVERLOAD_DETECTED)
	{
		fprintf(stderr, "adc overload\n");
		mir_sdr_GainChangeCallbackMessageReceived();
	}
	else if (gRdB == mir_sdr_ADC_OVERLOAD_CORRECTED)
	{
		fprintf(stderr, "adc overload corrected\n");
		mir_sdr_GainChangeCallbackMessageReceived();
	}
	else
	{
		if (verbose)
		{
			fprintf(stderr, "new gain reduction (%u), lna gain reduction (%u)\n", gRdB, lnaGRdB);
		}
	}
}


/*
 *****************
 * rx_callback() *
 *****************
 * Copies data from the SDRPlay RSP. (using 60% of a CPU on Pi2)
 *
 * Writes all samples into a circular buffer in Sixteen parts
 * The main thread reads buffers ASAP
 *
 */
int16_t *circ_buffer = NULL;
volatile uint32_t buff_now = 0;
uint32_t buff_offset = 0;
void rx_callback(short* xi, short* xq, unsigned int firstSampleNum, int grChanged, int rfChanged, int fsChanged, unsigned int numSamples, unsigned int reset, unsigned int hwRemoved, void* cbContext)
{
	if (circ_buffer && !do_exit) {
		unsigned int i, j;
		int16_t *data;

		// (sample_format == RSP_TCP_SAMPLE_FORMAT_INT16)
		j = buff_offset;
		data = &circ_buffer[(buff_now & 15) * FFT_SIZE * 2];
		for (i = 0; (i < numSamples) && (j < FFT_SIZE * 2); i++) {
			data[j++] = *xi;
			data[j++] = *xq;
			xi++; xq++;
		}
		buff_offset = j;
		if (i == numSamples)
			return;

		// need to start a new buffer part
		buff_offset = 0;
		buff_now++;  // sixteen buffer parts
		data = &circ_buffer[(buff_now & 15) * FFT_SIZE * 2];
		for (j = buff_offset; (i < numSamples) && (j < FFT_SIZE * 2); i++) {
			data[j++] = *xi;
			data[j++] = *xq;
			xi++; xq++;
		}
		buff_offset = j;
	}
}


// Downsample by 3 / 32 - 2048MHz to 96 Khz
uint32_t buff_out = 0;
void get_data()
{
	int i;
	uint32_t line = buff_out & 15; // get next line
	int16_t *in = &circ_buffer[line * FFT_SIZE * 2];
	int16_t *out = fm_buff;

	while (buff_out >= buff_now) {
		usleep(1000);
		if (do_exit)
			return;
	}

	for ( i = 0; (i + 63) < (2 * 2048); /* i+=64 */) {
		int ia, ib, qa, qb;
		ia = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qa = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		ib = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qb = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		*out++ = (ia + ib) >> 4;
		*out++ = (qa + qb) >> 4;

		i+=2; // skip a sample (i += 22)
		ia = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qa = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		ib = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qb = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		*out++ = (ia + ib) >> 4;
		*out++ = (qa + qb) >> 4;

		i+=2; // skip a sample (i += 44)
		ia = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qa = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		ib = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i++;
		qb = in[i] + in[i+2] + in[i+4] + in[i+6] + in[i+8];
		i+=9;
		*out++ = (ia + ib) >> 4;
		*out++ = (qa + qb) >> 4;

		// dont skip a sample (i += 64)
	}
	buff_out++;
}

static rsp_model_t hardware_ver_to_model(int hw_version)
{
	// Convert hardware version from library to internal enumerated type
	switch (hw_version)
	{
	case 1:
		return RSP_MODEL_RSP1;
	case 255:
		return RSP_MODEL_RSP1A;
	case 2:
		return RSP_MODEL_RSP2;
	case 3:
		return RSP_MODEL_RSPDUO;
	default:
		return RSP_MODEL_UNKNOWN;
	}
}

static rsp_band_t frequency_to_band(unsigned int f)
{
	if ( f < 60000000) {
		return current_antenna_input == 2 ? BAND_AM_HIZ : BAND_AM;
	}
	else
	if (f < 120000000)
	{
		return BAND_VHF;
	}
	else
	if (f < 250000000)
	{
		return BAND_3;
	}
	else
	if (f < 400000000)
	{
		return BAND_X;
	}
	else
	if (f < 420000000)
	{
		return SONDE;
	}
	else
	if (f < 1000000000)
	{
		return BAND_45;
	}
	else
	if (f <= 2000000000)
	{
		return BAND_L;
	}
	else
	{
		return BAND_UNKNOWN;
	}
}

static const char* model_to_string(rsp_model_t model)
{
	// Convert enumerated model to string for printing
	switch (model)
	{
	case RSP_MODEL_RSP1:
		return "RSP 1";
	case RSP_MODEL_RSP1A:
		return "RSP 1A";
	case RSP_MODEL_RSP2:
		return "RSP 2";
	case RSP_MODEL_RSPDUO:
		return "RSP DUO";
	default:
		return "Unknown";
	}
}

static rsp_capabilities_t *model_to_capabilities(rsp_model_t model)
{
	unsigned int i;
	for (i = 0; i < sizeof(device_caps) / sizeof(device_caps[0]); i++)
	{
		if (device_caps[i].model == model) {
			return &device_caps[i];
		}
	}

	return NULL;
}

static int gain_index_to_gain(unsigned int index, uint8_t *if_gr_out, uint8_t *lna_state_out)
{
	uint8_t max_gr = hardware_caps->max_ifgr;
	uint8_t min_gr = hardware_caps->min_ifgr;
	const uint8_t *if_gains;
	const uint8_t *lnastates;

	if_gains = NULL;
	lnastates = NULL;

	switch (current_band)
	{
	case BAND_AM:
		if_gains = hardware_caps->am_if_gains;
		lnastates = hardware_caps->am_lna_states;
		break;

	case BAND_VHF:
		if_gains = hardware_caps->vhf_if_gains;
		lnastates = hardware_caps->vhf_lna_states;
		break;

	case BAND_3:
		if_gains = hardware_caps->band3_if_gains;
		lnastates = hardware_caps->band3_lna_states;
		break;

	case BAND_X:
		if_gains = hardware_caps->bandx_if_gains;
		lnastates = hardware_caps->bandx_lna_states;
		break;

	case SONDE:
	case BAND_45:
		if_gains = hardware_caps->band45_if_gains;
		lnastates = hardware_caps->band45_lna_states;
		break;

	case BAND_L:
		if_gains = hardware_caps->lband_if_gains;
		lnastates = hardware_caps->lband_lna_states;
		break;

	case BAND_AM_HIZ:
		if_gains = hardware_caps->hiz_if_gains;
		lnastates = hardware_caps->hiz_lna_states;
		break;

	default:
		break;
	}

	if (if_gains && lnastates) {

		uint8_t if_gr = max_gr - (const_if_gain + if_gains[index]);
		if (if_gr < min_gr) {
			if_gr = min_gr;
		}

		*if_gr_out = if_gr;
		*lna_state_out = lnastates[index];
		return 0;
	}

	return 1;
}

static int apply_agc_settings()
{
	int r;
	mir_sdr_AgcControlT agc = agc_state ? mir_sdr_AGC_100HZ : mir_sdr_AGC_DISABLE;

	r = mir_sdr_AgcControl(agc, agc_set_point, 0, 0, 0, 0, lna_state);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "agc control error (%d)\n", r);
	}

	return r;
}

static int apply_gain_settings()
{
	int r;

	r = mir_sdr_RSP_SetGr(gain_reduction, lna_state, 1, 0);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "set gain reduction error (%d)\n", r);
	}

	return r;
}

static int set_if_gain_reduction(int gr)
{
	if (gr != gain_reduction && !agc_state) {
		gain_reduction = gr;
		apply_gain_settings();
	}

	return 0;
}

static int set_lna(unsigned int lnastate)
{
	if (lnastate != lna_state) {
		lna_state = lnastate;
		apply_gain_settings();
	}

	return 0;
}

static int set_agc(unsigned int enable)
{
	if (enable != agc_state) {
		agc_state = enable ? 1 : 0;
		apply_agc_settings();
	}

	return 0;
}

static int set_agc_setpoint(int set_point)
{
	if (set_point != agc_set_point) {
		agc_set_point = set_point;
		apply_agc_settings();
	}

	return 0;
}

static int set_bias_t(unsigned int enable)
{
	int r;
	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		r = mir_sdr_RSPII_BiasTControl(enable);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "bias-t control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSP1A:
		r = mir_sdr_rsp1a_BiasT(enable);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "bias-t control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		r = mir_sdr_rspDuo_BiasT(enable);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "bias-t control error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			fprintf(stderr, "bias-t not supported\n");
		}
		break;
	}

	return 0;
}

static int set_refclock_output(unsigned int enable)
{
	int r;
	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		r = mir_sdr_RSPII_ExternalReferenceControl(enable);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "external reference control error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		r = mir_sdr_rspDuo_ExtRef(enable);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "external reference control error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			fprintf(stderr, "reference clock output not supported\n");
		}
		break;
	}

	return 0;
}

static int set_antenna_input(unsigned int antenna)
{
	if (hardware_model == RSP_MODEL_RSP2 || hardware_model == RSP_MODEL_RSPDUO)
	{
		int r, new_am_port = 0;
		switch (antenna)
		{
		case RSP_TCP_ANTENNA_INPUT_A:

			if (hardware_model == RSP_MODEL_RSP2)
			{
				r = mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_A);
				if (r != mir_sdr_Success) {
					fprintf(stderr, "set antenna input error (%d)\n", r);
				}
			}
			else
				if (hardware_model == RSP_MODEL_RSPDUO)
				{
					r = mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_1);
					if (r != mir_sdr_Success) {
						fprintf(stderr, "set tuner error (%d)\n", r);
					}
				}

			new_am_port = 0;
			break;

		case RSP_TCP_ANTENNA_INPUT_B:

			if (hardware_model == RSP_MODEL_RSP2)
			{
				r = mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
				if (r != mir_sdr_Success) {
					fprintf(stderr, "set antenna input error (%d)\n", r);
				}
			}
			else
				if (hardware_model == RSP_MODEL_RSPDUO)
				{
					r = mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_2);
					if (r != mir_sdr_Success) {
						fprintf(stderr, "set tuner error (%d)\n", r);
					}
				}

			new_am_port = 0;
			break;

		case RSP_TCP_ANTENNA_INPUT_HIZ:

			if (hardware_model == RSP_MODEL_RSPDUO)
			{
				r = mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_1);
				if (r != mir_sdr_Success) {
					fprintf(stderr, "set tuner error (%d)\n", r);
				}
			}

			new_am_port = 1;
			break;
		}

		r = mir_sdr_AmPortSelect(new_am_port);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set am port select error (%d)\n", r);
		}

		am_port = new_am_port;
		current_antenna_input = antenna;

		rsp_band_t new_band = frequency_to_band(current_frequency);
		mir_sdr_ReasonForReinitT reason = mir_sdr_CHANGE_AM_PORT;
		if (new_band != current_band) {
			uint8_t if_gr, lnastate;

			current_band = new_band;

			gain_index_to_gain(last_gain_idx, &if_gr, &lnastate);			
			gain_reduction = if_gr;
			lna_state = lnastate;
			reason |= mir_sdr_CHANGE_GR;
		}

		r = mir_sdr_Reinit(&gain_reduction, 0.0, 0.0, mir_sdr_BW_Undefined,
			mir_sdr_IF_Undefined, mir_sdr_LO_Undefined,
			lna_state, &infoOverallGr, mir_sdr_USE_RSP_SET_GR,
			&samples_per_packet, reason);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "reinit error (%d)\n", r);
		}
	}
	else
	{
		if (verbose) {
			fprintf(stderr, "antenna input not supported\n");
		}
	}

	return 0;
}


static int set_notch_filters(unsigned int notch)
{
	int r;
	unsigned int rf_notch = (notch & RSP_TCP_NOTCH_RF) ? 1 : 0;
	unsigned int am_notch = (notch & RSP_TCP_NOTCH_AM) ? 1 : 0;
	unsigned int dab_notch = (notch & RSP_TCP_NOTCH_DAB) ? 1 : 0;
	unsigned int bc_notch = (notch & RSP_TCP_NOTCH_BROADCAST) ? 1 : 0;

	switch (hardware_model)
	{
	case RSP_MODEL_RSP2:
		r = mir_sdr_RSPII_RfNotchEnable(rf_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set rf notch error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSP1A:
		r = mir_sdr_rsp1a_DabNotch(dab_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set dab notch error (%d)\n", r);
		}
		r = mir_sdr_rsp1a_BroadcastNotch(bc_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set broadcast notch error (%d)\n", r);
		}
		break;

	case RSP_MODEL_RSPDUO:
		r = mir_sdr_rspDuo_DabNotch(dab_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set dab notch error (%d)\n", r);
		}
		r = mir_sdr_rspDuo_BroadcastNotch(bc_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set broadcast notch error (%d)\n", r);
		}		
		r = mir_sdr_rspDuo_Tuner1AmNotch(am_notch);
		if (r != mir_sdr_Success) {
			fprintf(stderr, "set am notch error (%d)\n", r);
		}
		break;

	default:
		if (verbose) {
			fprintf(stderr, "notch filter not supported\n");
		}
		break;
	}

	return 0;
}

static int set_gain_by_index(unsigned int index)
{
	int r;
	uint8_t if_gr, lnastate;

	if (index > GAIN_STEPS - 1) {
		fprintf(stderr, "gain step %d out of range", index);
		return 0;
	}

	if (gain_index_to_gain(index, &if_gr, &lnastate) != 0) {
		fprintf(stderr, "unable to get gain for current band\n");
		return 0;
	}

	gain_reduction = if_gr;
	lna_state = lnastate;

	r = mir_sdr_Reinit(&gain_reduction, 0.0, 0.0, mir_sdr_BW_Undefined,
		mir_sdr_IF_Undefined, mir_sdr_LO_Undefined,
		lna_state, &infoOverallGr, mir_sdr_USE_RSP_SET_GR,
		&samples_per_packet, mir_sdr_CHANGE_GR);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "set gain reduction returned (%d)\n", r);
	}

	apply_agc_settings();
	last_gain_idx = index;

	return r;
}

static int set_gain(unsigned int db)
{
	int p;
	unsigned int index;

	// quantise R820T gains in tenths of dB into indexes 
	p = ((9 + db) / 5);
	
	// clamp
	if (p > 100)
	{
		p = 100;
	}
	if (p < 0)
	{
		p = 0;
	}
	
	index = (unsigned int) (((GAIN_STEPS-1) / 100.0f) * p);
	return set_gain_by_index(index);
}

static int set_tuner_gain_mode(unsigned int mode)
{
	int r;

	if (mode) {
		r = mir_sdr_AgcControl(mir_sdr_AGC_DISABLE, agc_set_point, 0, 0, 0, 0, lna_state);
		set_gain_by_index(last_gain_idx);
		fprintf(stderr, "agc disabled\n");
	}
	else {
		r = mir_sdr_AgcControl(mir_sdr_AGC_100HZ, agc_set_point, 0, 0, 0, 0, lna_state);
		fprintf(stderr, "agc enabled\n");
	}

	if (r != mir_sdr_Success) {
		fprintf(stderr, "tuner gain (agc) control error (%d)\n", r);
	}

	return r;
}

static int set_freq_correction(int32_t corr)
{
	int r;

	r = mir_sdr_SetPpm((double)corr);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "set freq correction error (%d)\n", r);
	}

	return r;
}

static int set_freq(uint32_t f)
{
	int r;
	rsp_band_t new_band, old_band;

	old_band = current_band;
	new_band = frequency_to_band(f);

	current_frequency = f;
	current_band = new_band;

	r = mir_sdr_Reinit(&gain_reduction, 0.0, (double)f / 1e6, mir_sdr_BW_Undefined,
		mir_sdr_IF_Undefined, mir_sdr_LO_Undefined,
		lna_state, &infoOverallGr, mir_sdr_USE_RSP_SET_GR,
		&samples_per_packet, mir_sdr_CHANGE_RF_FREQ);

	if (r != mir_sdr_Success) {
		if (verbose) {
			fprintf(stderr, "set freq returned (%d)\n", r);
		}
	}

	apply_agc_settings();

	// Reapply valid gain for new band
	if (old_band != new_band) {
		set_gain_by_index(last_gain_idx);
	}

	return r;
}

static int set_sample_rate(uint32_t sr)
{
	int r;
	double f;
	int decimation;

	if (sr < (2000000 / MAX_DECIMATION_FACTOR) || sr > 10000000) {
		fprintf(stderr, "sample rate %u is not supported\n", sr);
		return -1;
	}

	decimation = 1;
	if (sr < 2000000)
	{
		int c = 0;

		// Find best decimation factor
		while (sr * (1 << c) < 2000000 && (1 << c) < MAX_DECIMATION_FACTOR) {
			c++;
		}

		decimation = 1 << c;

		if (sr >= 1536000 && sr < 2000000)
		{
			bwType = mir_sdr_BW_1_536;
		}
		else
		if (sr >= 600000 && sr < 1536000)
		{
			bwType = mir_sdr_BW_0_600;
		}
		else
		if (sr >= 300000 && sr < 600000)
		{
			bwType = mir_sdr_BW_0_300;
		}
		else
		{
			bwType = mir_sdr_BW_0_200;
		}
	}
	else
	{
		if (sr >= 8000000 && sr <= 10000000)
		{
			bwType = mir_sdr_BW_8_000;
		}
		else
		if (sr >= 7000000 && sr < 8000000)
		{
			bwType = mir_sdr_BW_7_000;
		}
		else
		if (sr >= 6000000 && sr < 7000000)
			{
				bwType = mir_sdr_BW_6_000;
			}
		else
		if (sr >= 5000000 && sr < 6000000)
		{
			bwType = mir_sdr_BW_5_000;
		}
		else
		if (sr >= 2500000 && sr < 5000000)
		{
			decimation = 2;
			bwType = mir_sdr_BW_1_536;
		}
		else
		{
			bwType = mir_sdr_BW_1_536;
		}
	}

	f = (double)(sr * decimation);

	if (decimation == 1) {
		mir_sdr_DecimateControl(0, 0, 0);
	}
	else {
		mir_sdr_DecimateControl(1, decimation, 1);
	}

	fprintf(stderr, "device SR %.2f, decim %d, output SR %u, IF Filter BW %d kHz\n", f, decimation, sr, bwType);

	r = mir_sdr_Reinit(&gain_reduction, (double)f / 1e6, 0, bwType,
		mir_sdr_IF_Undefined, mir_sdr_LO_Undefined,
		lna_state, &infoOverallGr, mir_sdr_USE_RSP_SET_GR,
		&samples_per_packet,
		mir_sdr_CHANGE_FS_FREQ | mir_sdr_CHANGE_BW_TYPE);

	if (r != mir_sdr_Success) {
		fprintf(stderr, "set sample rate error (%d)\n", r);
	}

	return r;
}

int init_rsp_device(unsigned int sr, unsigned int freq, int enable_bias_t, unsigned int notch, int enable_refout, int antenna, int gain)
{
	int r;
	uint8_t ifgain, lnastate;

	// initialise frequency state
	current_band = frequency_to_band(freq);
	current_frequency = freq;

	// initialise gain
	if (!gain_index_to_gain(gain, &ifgain, &lnastate)) {
		gain_reduction = ifgain;
		lna_state = lnastate;
	}

	r = mir_sdr_StreamInit(&gain_reduction, (sr / 1e6), (freq / 1e6), bwType, mir_sdr_IF_Zero,
		lna_state, &infoOverallGr, mir_sdr_USE_RSP_SET_GR,
		&samples_per_packet, rx_callback, gc_callback, (void *)NULL);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "failed to start the RSP device, return (%d)\n", r);
		return -1;
	}

	// set bias-T
	set_bias_t(enable_bias_t);

	// set notch filters
	set_notch_filters(notch);

	// set external reference output
	set_refclock_output(enable_refout);

	// set antenna input
	set_antenna_input(antenna);

	return 0;
}

/**********************************/

void push_to_file()
{
	static int ptr = 0;

	memcpy( &result48[ptr * LEN_FM_N], fm_buff, LEN_FM_N * sizeof(int16_t) );
	ptr++;
	if (ptr > 9) {
		ptr = 0;
		fwrite(result48, 2, LEN_OUT, file);
	}
}

// inplace downsample interleaved IQ data
void quartersampleIQ(int16_t *data, int length)
{
	int i, f;
	for (i = 0; (i*4) < (2 * length - 7); i++) {
		f = i * 4;
		data[i] = (data[f] + 2 * (data[f + 2] + data[f + 4])
			+ data[f + 6] ) >> 3;
		f++; i++;
		data[i] = ( data[f] + 2 * (data[f + 2] + data[f + 4])
			+ data[f + 6] ) >> 3;
	}
}

// inplace downsample single channel data
void quartersample(int16_t *data, int length)
{
	int i, f;
	for (i = 0; (i*4)<(length - 3); i++) {
		f = i * 4;
		data[i] = (data[f] + 2 * (data[f + 1]+ data[f + 2])
			+ data[f + 3] ) >> 3;
	}
}

void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

int16_t polar_disc(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;

	// multiply(ar, aj, br, -bj, &cr, &cj);
	cr = ar*br + aj*bj;
	cj = aj*br - ar*bj;

	angle = atan2((double)cj, (double)cr);
	return (int16_t)(angle * (1 << 14) / 3.2);
}

int fm_pre_r, fm_pre_j;
void fm_demod(int wide)
{
	int i, len;
	int16_t *lp = fm_buff;
	int16_t *r = fm_buff;

	if (wide) {
		len = LEN_FM_W;
	} else {
		quartersampleIQ(lp, LEN_FM_W);
		len = LEN_FM_N;
	}

	r[0] = polar_disc(lp[0], lp[1], fm_pre_r, fm_pre_j);

	for (i = 1; i < len / 2; i++) {
		r[i] = polar_disc(lp[i*2], lp[i*2+1], lp[i*2-2], lp[i*2-1]);
	}
	fm_pre_r = lp[len - 2];
	fm_pre_j = lp[len - 1];

	if (wide)
		quartersample(r, LEN_FM_W);

	push_to_file();
}

void usb_demod()
{
	int i, pcm;
	int16_t *lp = fm_buff;
	int16_t *r  = fm_buff;

	quartersampleIQ(lp, LEN_FM_W);
	for (i = 0; i < LEN_FM_N / 2; i ++) {
		pcm = (lp[i*2] + lp[i*2+1]);
		r[i] = pcm >> 2;
	}
	push_to_file();
}

void generate_header(int samplerate)
{
	int i, s_rate, b_rate;
	char *channels = "\1\0";
	char *align = "\2\0";
	uint8_t samp_rate[4] = {0, 0, 0, 0};
	uint8_t byte_rate[4] = {0, 0, 0, 0};
	s_rate = samplerate;
	b_rate = s_rate * 2;
	if (mode_demod == MODE_RAW) {
		channels = "\2\0";
		align = "\4\0";
		b_rate *= 2;
	}
	for (i=0; i<4; i++) {
		samp_rate[i] = (uint8_t)((s_rate >> (8*i)) & 0xFF);
		byte_rate[i] = (uint8_t)((b_rate >> (8*i)) & 0xFF);
	}
	fwrite("RIFF",     1, 4, file);
	fwrite("\xFF\xFF\xFF\xFF", 1, 4, file);  /* size */
	fwrite("WAVE",     1, 4, file);
	fwrite("fmt ",     1, 4, file);
	fwrite("\x10\0\0\0", 1, 4, file);  /* size */
	fwrite("\1\0",     1, 2, file);  /* pcm */
	fwrite(channels,   1, 2, file);
	fwrite(samp_rate,  1, 4, file);
	fwrite(byte_rate,  1, 4, file);
	fwrite(align, 1, 2, file);
	fwrite("\x10\0",     1, 2, file);  /* bits per channel */
	fwrite("data",     1, 4, file);
	fwrite("\xFF\xFF\xFF\xFF", 1, 4, file);  /* size */
}


// Sample 2.048 Mhz, take FM at 192k wide, or 48k narrow
void usage(void)
{
	printf("rsp_fm, a minimal rtl_fm implementation for SDRPlay receivers."
		"\n\n"
		"Usage: rsp_fm -f 88.5M [options] filename\n"
		"\t[-f frequency (no scanning support)]\n"
		"\t[-g gain (0.0 to 56.0, default: 32)]\n"
		"\t[-W sets wideband, for broadcast FM]\n"
		"\n"
		"\t[-d RSP device to use (default: 1, first found)]\n"
		"\t[-A Antenna Port select* (0/1/2, default: 0, Port A)]\n"
		"\t[-T Bias-T enable* (default: disabled)]\n"
		"\t[-R Refclk output enable* (default: disabled)]\n"
		"\n"
		"\t Writes samples to file with a WAV header\n"
		"\t (** only supports 48kHz output **)\n"
		"\n rsp_fm -f 88.5M -W | aplay"
		"\n"
		"\n"
	      );
		exit(1);
}

int main(int argc, char **argv)
{
	int length, r, opt, wb_mode = 0;
	uint32_t i, frequency = DEFAULT_FREQUENCY, samp_rate = DEFAULT_SAMPLERATE;

	float ver;
	mir_sdr_DeviceT devices[MAX_DEVS];
	unsigned int numDevs;
	unsigned int notch = 0;
	int devAvail = 0;
	int device = 0;
	int antenna = 0;
	int enable_biastee = 0;
	int enable_refout = 0;
	int bit_depth = 16;
	int gain = DEFAULT_GAIN;
	int widefm = 0;

	struct sigaction sigact, sigign;
	char *filename;

	fprintf(stderr, "rsp_fm V%d.%d\n\n", RSP_FM_VERSION_MAJOR, RSP_FM_VERSION_MINOR);

	while ((opt = getopt(argc, argv, "f:g:A:d:P:p:F:M:WTRDOW")) != -1) {
		switch (opt) {
		case 'd':
			device = atoi(optarg) - 1;
			break;
		case 'g':
			//gain range is 0 - 28 instead of 0.0 to 50.0 for rtl_sdr
			gain = (int)(atof(optarg) / 2.0);
			if (gain < 0) { // autogain request
				gain = DEFAULT_GAIN;
			} else if (gain > MAX_GAIN) // out of range
				gain = MAX_GAIN;
			break;
		case 'A':
			antenna = atoi(optarg);
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'R':
			enable_refout = 1;
			break;
		case 'p':
			// ppm_error = atoi(optarg);
			break;
		case 'D':
			// direct_sampling = 1;
		case 'O':
			// offset_tuning = 1;
		case 'F':
			// comp_fir_size = atoi(optarg);
			break;
		case 'M':
			if (strcmp("fm",  optarg) == 0) {
				mode_demod = MODE_FM;	}
			else if (strcmp("raw",  optarg) == 0) {
				mode_demod = MODE_RAW;	}
			else if (strcmp("am",  optarg) == 0) {
				mode_demod = MODE_AM;	}
			else if (strcmp("usb", optarg) == 0) {
				mode_demod = MODE_USB;	}
			else if (strcmp("lsb", optarg) == 0) {
				mode_demod = MODE_LSB;	}
			else if (strcmp("wbfm",  optarg) == 0) {
				widefm = 1;	}
			break;
		case 'W':
			widefm = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		file = stdout;
	} else {
		filename = argv[optind];
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			exit(1);
		}
	}

	freq = frequency;
	rate = samp_rate;
	sample_format = RSP_TCP_SAMPLE_FORMAT_INT16;
	generate_header(48000);

	// check API version
	r = mir_sdr_ApiVersion(&ver);
	if (ver != MIR_SDR_API_VERSION) {
		//  Error detected, include file does not match dll. Deal with error condition.
		fprintf(stderr, "library libmirsdrapi-rsp must be version %.2f\n", ver);
		exit(1);
	}
	// fprintf(stderr, "libmirsdrapi-rsp version %.2f found\n", ver);
#if 0
	// enable debug output
	if (verbose) {
		mir_sdr_DebugEnable(1);
	}
#endif
	// select RSP device
	r = mir_sdr_GetDevices(&devices[0], &numDevs, MAX_DEVS);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "Failed to get device list (%d)\n", r);
		exit(1);
	}

	for (i = 0; i < numDevs; i++) {
		if (devices[i].devAvail == 1) {
			devAvail++;
		}
	}

	if (devAvail == 0) {
		fprintf(stderr, "no RSP devices available.\n");
		exit(1);
	}

	if (devices[device].devAvail != 1) {
		fprintf(stderr, "RSP selected (%d) is not available.\n", (device + 1));
		exit(1);
	}

	r = mir_sdr_SetDeviceIdx(device);
	if (r != mir_sdr_Success) {
		fprintf(stderr, "Failed to set device index (%d)\n", r);
		exit(1);
	}

	// get RSP model
	hardware_version = devices[device].hwVer;
	hardware_model = hardware_ver_to_model(hardware_version);
	hardware_caps = model_to_capabilities(hardware_model);

	if (hardware_model == RSP_MODEL_UNKNOWN || hardware_caps == NULL) {
		fprintf(stderr, "unknown RSP model (hw ver %d)\n", hardware_version);
	}
	else {
		fprintf(stderr, "detected RSP model '%s' (hw ver %d)\n", model_to_string(hardware_model), hardware_version);
	}

	// enable DC offset and IQ imbalance correction
	mir_sdr_DCoffsetIQimbalanceControl(0, 0);
	// disable decimation and  set decimation factor to 1
	mir_sdr_DecimateControl(0, 1, 0);

	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);

	// allocate 16 buffers for 16 bit IQ samples
	circ_buffer = calloc(16, FFT_SIZE * 2 * sizeof(int16_t));

	// initialise API and start the rx
	r = init_rsp_device(samp_rate, frequency, enable_biastee, notch, enable_refout, antenna, gain);
	if (r != 0) {
		fprintf(stderr, "failed to initialise RSP device\n");
		goto out;
	}

	while (!do_exit) {
		get_data();
		if (mode_demod == MODE_FM) {
			fm_demod(widefm);
		} else if (mode_demod == MODE_USB) {
			usb_demod();
		}
	}

out:
	do_exit = 1;

	mir_sdr_StreamUninit();
	mir_sdr_ReleaseDeviceIdx();

	if (circ_buffer)
		free(circ_buffer);

	return 0;
}
