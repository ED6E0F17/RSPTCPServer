/* SDRPlay RSP settings and constants */

// *************************************
#define GAIN_STEPS (29)

const uint8_t rsp1_am_gains_lnastates[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_am_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 20, 21, 23, 7, 9, 11, 14, 15, 18, 19, 20, 22, 0, 1, 1, 2, 5, 7 };
const uint8_t rsp1_vhf_gains_lnastates[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_vhf_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 20, 21, 23, 7, 9, 11, 14, 15, 18, 19, 20, 22, 0, 1, 1, 2, 5, 7 };
const uint8_t rsp1_band3_gains_lnastates[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_band3_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 20, 21, 23, 7, 9, 11, 14, 15, 18, 19, 20, 22, 0, 1, 1, 2, 5, 7 };
const uint8_t rsp1_bandx_gains_lnastates[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_bandx_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 20, 21, 23, 7, 9, 11, 14, 15, 18, 19, 20, 22, 0, 1, 1, 2, 5, 7 };
const uint8_t rsp1_band45_gains_lnastates[] = { 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_band45_gains_ifgains[] = { 0, 1, 2, 3, 4, 1, 2, 6, 8, 9, 10, 1, 2, 4, 0, 2, 4, 7, 8, 11, 12, 13, 15, 17, 18, 18, 19, 22, 24 };
const uint8_t rsp1_lband_gains_lnastates[] = { 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_lband_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 1, 2, 4, 2, 4, 6, 9, 10, 13, 14, 15, 17, 19, 20, 20, 21, 24, 26 };

const uint8_t rsp1a_am_gains_lnastates[] = { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp1a_am_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 2, 4, 6, 9, 10, 13, 14, 15, 17, 0, 1, 1, 2, 5, 1 };
const uint8_t rsp1a_vhf_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rsp1a_vhf_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rsp1a_band3_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rsp1a_band3_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rsp1a_bandx_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rsp1a_bandx_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rsp1a_band45_gains_lnastates[] = { 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3 };
const uint8_t rsp1a_band45_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 5, 2, 3, 0, 1, 2, 4, 6, 0, 0, 0, 3, 5 };
const uint8_t rsp1a_lband_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp1a_lband_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 2, 2, 3, 6, 0 };

const uint8_t rsp2_am_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 5, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp2_am_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 0, 3, 4, 7, 8, 9, 1, 0, 1, 1, 2, 5, 1 };
const uint8_t rsp2_vhf_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 5, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp2_vhf_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 0, 3, 4, 7, 8, 9, 1, 0, 1, 1, 2, 5, 1 };
const uint8_t rsp2_band3_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 5, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp2_band3_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 0, 3, 4, 7, 8, 9, 1, 0, 1, 1, 2, 5, 1 };
const uint8_t rsp2_bandx_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 5, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rsp2_bandx_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 0, 3, 4, 7, 8, 9, 1, 0, 1, 1, 2, 5, 1 };
const uint8_t rsp2_band45_gains_lnastates[] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_band45_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 2, 4, 6, 2, 0, 3, 4, 5, 0, 2, 3, 3, 4, 7, 9 };
const uint8_t rsp2_lband_gains_lnastates[] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_lband_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 7, 8, 10, 13, 15, 1, 4, 0, 3, 4, 5, 7, 9, 10, 10, 11, 14, 16 };
const uint8_t rsp2_hiz_gains_lnastates[] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_hiz_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 5, 2, 3, 0, 1, 2, 4, 6, 7, 7, 8, 11, 13 };

const uint8_t rspduo_am_gains_lnastates[] = { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rspduo_am_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 2, 4, 6, 9, 10, 13, 14, 15, 17, 0, 1, 1, 2, 5, 1 };
const uint8_t rspduo_vhf_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rspduo_vhf_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rspduo_band3_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rspduo_band3_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rspduo_bandx_gains_lnastates[] = { 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 3, 2 };
const uint8_t rspduo_bandx_gains_ifgains[] = { 0, 1, 2, 3, 4, 3, 4, 8, 10, 11, 12, 15, 16, 18, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 0, 0, 1, 4, 0 };
const uint8_t rspduo_band45_gains_lnastates[] = { 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3 };
const uint8_t rspduo_band45_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 5, 2, 3, 0, 1, 2, 4, 6, 0, 0, 0, 3, 5 };
const uint8_t rspduo_lband_gains_lnastates[] = { 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2 };
const uint8_t rspduo_lband_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 2, 4, 0, 3, 4, 1, 2, 3, 5, 1, 2, 2, 3, 6, 0 };
const uint8_t rspduo_hiz_gains_lnastates[] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_hiz_gains_ifgains[] = { 0, 1, 2, 3, 4, 8, 9, 13, 15, 16, 17, 1, 2, 4, 1, 3, 5, 2, 3, 0, 1, 2, 4, 6, 7, 7, 8, 11, 13 };

typedef enum {
	RSP_MODEL_UNKNOWN = 0,
	RSP_MODEL_RSP1 = 1,
	RSP_MODEL_RSP1A = 2,
	RSP_MODEL_RSP2 = 3,
	RSP_MODEL_RSPDUO = 4
} rsp_model_t;

typedef enum {
	BAND_UNKNOWN = 0,
	BAND_AM = 1,
	BAND_VHF = 2,
	BAND_3 = 3,
	BAND_X = 4,
	BAND_45 = 5,
	BAND_L = 6,
	BAND_AM_HIZ = 7,
	SONDE = 8
} rsp_band_t;

typedef struct {
	rsp_model_t model;
	char *name;
	uint8_t antenna_input_count;
	uint8_t tuner_count;
	uint32_t capabilities;

	uint8_t min_ifgr;
	uint8_t max_ifgr;

	const uint8_t* am_lna_states;
	const uint8_t* am_if_gains;
	const uint8_t* vhf_lna_states;
	const uint8_t* vhf_if_gains;
	const uint8_t* band3_lna_states;
	const uint8_t* band3_if_gains;
	const uint8_t* bandx_lna_states;
	const uint8_t* bandx_if_gains;
	const uint8_t* band45_lna_states;
	const uint8_t* band45_if_gains;
	const uint8_t* lband_lna_states;
	const uint8_t* lband_if_gains;
	const uint8_t* hiz_lna_states;
	const uint8_t* hiz_if_gains;
} rsp_capabilities_t;

static rsp_capabilities_t device_caps[] = {
	{
		.model = RSP_MODEL_RSP1,
		.name = "RSP1",
		.antenna_input_count = 1,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1_am_gains_lnastates,
		.am_if_gains = rsp1_am_gains_ifgains,
		.vhf_lna_states = rsp1_vhf_gains_lnastates,
		.vhf_if_gains = rsp1_vhf_gains_ifgains,
		.band3_lna_states = rsp1_band3_gains_lnastates,
		.band3_if_gains = rsp1_band3_gains_ifgains,
		.bandx_lna_states = rsp1_bandx_gains_lnastates,
		.bandx_if_gains = rsp1_bandx_gains_ifgains,
		.band45_lna_states = rsp1_band45_gains_lnastates,
		.band45_if_gains = rsp1_band45_gains_ifgains,
		.lband_lna_states = rsp1_lband_gains_lnastates,
		.lband_if_gains = rsp1_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP1A,
		.name = "RSP1A",
		.antenna_input_count = 1,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1a_am_gains_lnastates,
		.am_if_gains = rsp1a_am_gains_ifgains,
		.vhf_lna_states = rsp1a_vhf_gains_lnastates,
		.vhf_if_gains = rsp1a_vhf_gains_ifgains,
		.band3_lna_states = rsp1a_band3_gains_lnastates,
		.band3_if_gains = rsp1a_band3_gains_ifgains,
		.bandx_lna_states = rsp1a_bandx_gains_lnastates,
		.bandx_if_gains = rsp1a_bandx_gains_ifgains,
		.band45_lna_states = rsp1a_band45_gains_lnastates,
		.band45_if_gains = rsp1a_band45_gains_ifgains,
		.lband_lna_states = rsp1a_lband_gains_lnastates,
		.lband_if_gains = rsp1a_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP2,
		.name = "RSP2",
		.antenna_input_count = 3,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_RF_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp2_am_gains_lnastates,
		.am_if_gains = rsp2_am_gains_ifgains,
		.vhf_lna_states = rsp2_vhf_gains_lnastates,
		.vhf_if_gains = rsp2_vhf_gains_ifgains,
		.band3_lna_states = rsp2_band3_gains_lnastates,
		.band3_if_gains = rsp2_band3_gains_ifgains,
		.bandx_lna_states = rsp2_bandx_gains_lnastates,
		.bandx_if_gains = rsp2_bandx_gains_ifgains,
		.band45_lna_states = rsp2_band45_gains_lnastates,
		.band45_if_gains = rsp2_band45_gains_ifgains,
		.lband_lna_states = rsp2_lband_gains_lnastates,
		.lband_if_gains = rsp2_lband_gains_ifgains,
		.hiz_lna_states = rsp2_hiz_gains_lnastates,
		.hiz_if_gains = rsp2_hiz_gains_ifgains,
	},
	{
		.model = RSP_MODEL_RSPDUO,
		.name = "RSPDUO",
		.antenna_input_count = 3,
		.tuner_count = 2,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_AM_NOTCH |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rspduo_am_gains_lnastates,
		.am_if_gains = rspduo_am_gains_ifgains,
		.vhf_lna_states = rspduo_vhf_gains_lnastates,
		.vhf_if_gains = rspduo_vhf_gains_ifgains,
		.band3_lna_states = rspduo_band3_gains_lnastates,
		.band3_if_gains = rspduo_band3_gains_ifgains,
		.bandx_lna_states = rspduo_bandx_gains_lnastates,
		.bandx_if_gains = rspduo_bandx_gains_ifgains,
		.band45_lna_states = rspduo_band45_gains_lnastates,
		.band45_if_gains = rspduo_band45_gains_ifgains,
		.lband_lna_states = rspduo_lband_gains_lnastates,
		.lband_if_gains = rspduo_lband_gains_ifgains,
		.hiz_lna_states = rspduo_hiz_gains_lnastates,
		.hiz_if_gains = rspduo_hiz_gains_ifgains,
	},
};


// *************************************

