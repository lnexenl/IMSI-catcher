//
// Created by lnex on 2021/4/9.
//
#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include "srslte/common/crash_handler.h"
#include "srslte/common/gen_mch_tables.h"
#include "srslte/phy/io/filesink.h"
#include "srslte/srslte.h"

#include "si_acquire.h"


#define PRINT_LINE_INIT()                                                                                              \
  int        this_nof_lines = 0;                                                                                       \
  static int prev_nof_lines = 0
#define PRINT_LINE(_fmt, ...)                                                                                          \
  printf("\033[K" _fmt "\n", ##__VA_ARGS__);                                                                           \
  this_nof_lines++
#define PRINT_LINE_RESET_CURSOR()                                                                                      \
  printf("\033[%dA", this_nof_lines);                                                                                  \
  prev_nof_lines = this_nof_lines
#define PRINT_LINE_ADVANCE_CURSOR() printf("\033[%dB", prev_nof_lines + 1)

#define ENABLE_AGC_DEFAULT

#ifndef DISABLE_RF



cell_search_cfg_t cell_detect_config = {.max_frames_pbch      = SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
        .max_frames_pss       = SRSLTE_DEFAULT_MAX_FRAMES_PSS,
        .nof_valid_pss_frames = SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
        .init_agc             = 0,
        .force_tdd            = false};

#else
#pragma message "Compiling pdsch_ue with no RF support"
#endif


typedef struct {
    int      nof_subframes;
    int      cpu_affinity;
    bool     disable_plots;
    bool     disable_plots_except_constellation;
    bool     disable_cfo;
    uint32_t time_offset;
    int      force_N_id_2;
    uint16_t rnti;
    char*    input_file_name;
    int      file_offset_time;
    float    file_offset_freq;
    uint32_t file_nof_prb;
    uint32_t file_nof_ports;
    uint32_t file_cell_id;
    bool     enable_cfo_ref;
    char*    estimator_alg;
    char*    rf_dev;
    char*    rf_args;
    uint32_t rf_nof_rx_ant;
    double   rf_freq;
    float    rf_gain;
    int      net_port;
    char*    net_address;
    int      net_port_signal;
    char*    net_address_signal;
    int      decimate;
    int32_t  mbsfn_area_id;
    uint8_t  non_mbsfn_region;
    uint8_t  mbsfn_sf_mask;
    int      tdd_special_sf;
    int      sf_config;
    int      verbose;
    bool     enable_256qam;
} prog_args_t;

void args_default(prog_args_t* args)
{
    args->disable_plots                      = false;
    args->disable_plots_except_constellation = false;
    args->nof_subframes                      = -1;
    args->rnti                               = SRSLTE_SIRNTI;
    args->force_N_id_2                       = -1; // Pick the best
    args->tdd_special_sf                     = -1;
    args->sf_config                          = -1;
    args->input_file_name                    = NULL;
    args->disable_cfo                        = false;
    args->time_offset                        = 0;
    args->file_nof_prb                       = 25;
    args->file_nof_ports                     = 1;
    args->file_cell_id                       = 0;
    args->file_offset_time                   = 0;
    args->file_offset_freq                   = 0;
    args->rf_dev                             = "";
    args->rf_args                            = "";
    args->rf_freq                            = -1.0;
    args->rf_nof_rx_ant                      = 1;
    args->enable_cfo_ref                     = false;
    args->estimator_alg                      = "interpolate";
    args->enable_256qam                      = false;
#ifdef ENABLE_AGC_DEFAULT
    args->rf_gain = -1.0;
#else
    args->rf_gain = 50.0;
#endif
    args->net_port           = -1;
    args->net_address        = "127.0.0.1";
    args->net_port_signal    = -1;
    args->net_address_signal = "127.0.0.1";
    args->decimate           = 0;
    args->cpu_affinity       = -1;
    args->mbsfn_area_id      = -1;
    args->non_mbsfn_region   = 2;
    args->mbsfn_sf_mask      = 32;
}

void usage(prog_args_t* args, char* prog)
{
    printf("Usage: %s [adgpPoOcildFRDnruMNvTG] -f rx_frequency (in Hz) | -i input_file\n", prog);
#ifndef DISABLE_RF
    printf("\t-I RF dev [Default %s]\n", args->rf_dev);
    printf("\t-a RF args [Default %s]\n", args->rf_args);
    printf("\t-A Number of RX antennas [Default %d]\n", args->rf_nof_rx_ant);
#ifdef ENABLE_AGC_DEFAULT
    printf("\t-g RF fix RX gain [Default AGC]\n");
#else
    printf("\t-g Set RX gain [Default %.1f dB]\n", args->rf_gain);
#endif
#else
    printf("\t   RF is disabled.\n");
#endif
    printf("\t-i input_file [Default use RF board]\n");
    printf("\t-o offset frequency correction (in Hz) for input file [Default %.1f Hz]\n", args->file_offset_freq);
    printf("\t-O offset samples for input file [Default %d]\n", args->file_offset_time);
    printf("\t-p nof_prb for input file [Default %d]\n", args->file_nof_prb);
    printf("\t-P nof_ports for input file [Default %d]\n", args->file_nof_ports);
    printf("\t-c cell_id for input file [Default %d]\n", args->file_cell_id);
    printf("\t-r RNTI in Hex [Default 0x%x]\n", args->rnti);
    printf("\t-l Force N_id_2 [Default best]\n");
    printf("\t-C Disable CFO correction [Default %s]\n", args->disable_cfo ? "Disabled" : "Enabled");
    printf("\t-F Enable RS-based CFO correction [Default %s]\n", !args->enable_cfo_ref ? "Disabled" : "Enabled");
    printf("\t-R Channel estimates algorithm (average, interpolate, wiener) [Default %s]\n", args->estimator_alg);
    printf("\t-t Add time offset [Default %d]\n", args->time_offset);
    printf("\t-T Set TDD special subframe configuration [Default %d]\n", args->tdd_special_sf);
    printf("\t-G Set TDD uplink/downlink configuration [Default %d]\n", args->sf_config);
#ifdef ENABLE_GUI
    printf("\t-d disable plots [Default enabled]\n");
  printf("\t-D disable all but constellation plots [Default enabled]\n");
#else  /* ENABLE_GUI */
    printf("\t plots are disabled. Graphics library not available\n");
#endif /* ENABLE_GUI */
    printf("\t-y set the cpu affinity mask [Default %d] \n  ", args->cpu_affinity);
    printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
    printf("\t-s remote UDP port to send input signal (-1 does nothing with it) [Default %d]\n", args->net_port_signal);
    printf("\t-S remote UDP address to send input signal [Default %s]\n", args->net_address_signal);
    printf("\t-u remote TCP port to send data (-1 does nothing with it) [Default %d]\n", args->net_port);
    printf("\t-U remote TCP address to send data [Default %s]\n", args->net_address);
    printf("\t-M MBSFN area id [Default %d]\n", args->mbsfn_area_id);
    printf("\t-N Non-MBSFN region [Default %d]\n", args->non_mbsfn_region);
    printf("\t-q Enable/Disable 256QAM modulation (default %s)\n", args->enable_256qam ? "enabled" : "disabled");
    printf("\t-v [set srslte_verbose to debug, default none]\n");
}

void parse_args(prog_args_t* args, int argc, char** argv)
{
    int opt;
    args_default(args);

    while ((opt = getopt(argc, argv, "adAogliIpPcOCtdDFRqnvrfuUsSZyMNBTG")) != -1) {
        switch (opt) {
            case 'i':
                args->input_file_name = argv[optind];
                break;
            case 'p':
                args->file_nof_prb = (uint32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'P':
                args->file_nof_ports = (uint32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'o':
                args->file_offset_freq = strtof(argv[optind], NULL);
                break;
            case 'O':
                args->file_offset_time = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'c':
                args->file_cell_id = (uint32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'I':
                args->rf_dev = argv[optind];
                break;
            case 'a':
                args->rf_args = argv[optind];
                break;
            case 'A':
                args->rf_nof_rx_ant = (uint32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'g':
                args->rf_gain = strtof(argv[optind], NULL);
                break;
            case 'C':
                args->disable_cfo = true;
                break;
            case 'F':
                args->enable_cfo_ref = true;
                break;
            case 'R':
                args->estimator_alg = argv[optind];
                break;
            case 't':
                args->time_offset = (uint32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'f':
                args->rf_freq = strtod(argv[optind], NULL);
                break;
            case 'T':
                args->tdd_special_sf = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'G':
                args->sf_config = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'n':
                args->nof_subframes = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'r':
                args->rnti = strtol(argv[optind], NULL, 16);
                break;
            case 'l':
                args->force_N_id_2 = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'u':
                args->net_port = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'U':
                args->net_address = argv[optind];
                break;
            case 's':
                args->net_port_signal = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'S':
                args->net_address_signal = argv[optind];
                break;
            case 'd':
                args->disable_plots = true;
                break;
            case 'D':
                args->disable_plots_except_constellation = true;
                break;
            case 'v':
                srslte_verbose++;
                args->verbose = srslte_verbose;
                break;
            case 'Z':
                args->decimate = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'y':
                args->cpu_affinity = (int)strtol(argv[optind], NULL, 10);
                break;
            case 'M':
                args->mbsfn_area_id = (int32_t)strtol(argv[optind], NULL, 10);
                break;
            case 'N':
                args->non_mbsfn_region = (uint8_t)strtol(argv[optind], NULL, 10);
                break;
            case 'B':
                args->mbsfn_sf_mask = (uint8_t)strtol(argv[optind], NULL, 10);
                break;
            case 'q':
                args->enable_256qam ^= true;
                break;
            default:
                usage(args, argv[0]);
                exit(-1);
        }
    }
    if (args->rf_freq < 0 && args->input_file_name == NULL) {
        usage(args, argv[0]);
        exit(-1);
    }
}


uint8_t* data[SRSLTE_MAX_CODEWORDS];

bool go_exit = false;

void sig_int_handler(int signo)
{
    printf("SIGINT received. Exiting...\n");
    if (signo == SIGINT) {
        go_exit = true;
    } else if (signo == SIGSEGV) {
        exit(1);
    }
}

cf_t* sf_buffer[SRSLTE_MAX_PORTS] = {NULL};

int srslte_rf_recv_wrapper(void* h, cf_t* data_[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t* t)
{
    DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
    void* ptr[SRSLTE_MAX_PORTS];
    for (int i = 0; i < SRSLTE_MAX_PORTS; i++) {
        ptr[i] = data_[i];
    }
    return srslte_rf_recv_with_time_multi((srslte_rf_t*)h, ptr, nsamples, true, NULL, NULL);
}

static SRSLTE_AGC_CALLBACK(srslte_rf_set_rx_gain_th_wrapper_)
{
    srslte_rf_set_rx_gain_th((srslte_rf_t*)h, gain_db);
}

extern float mean_exec_time;

enum receiver_state { DECODE_MIB, DECODE_PDSCH } state;

srslte_cell_t      cell;
srslte_ue_dl_t     ue_dl;
srslte_ue_dl_cfg_t ue_dl_cfg;
srslte_dl_sf_cfg_t dl_sf;
srslte_pdsch_cfg_t pdsch_cfg;
srslte_ue_sync_t   ue_sync;
prog_args_t        prog_args;

uint32_t pkt_errors = 0, pkt_total = 0, nof_detected = 0, pmch_pkt_errors = 0, pmch_pkt_total = 0, nof_trials = 0;

//srslte_netsink_t net_sink, net_sink_signal;

int main(int argc, char** argv) {
    int ret;

    srslte_rf_t rf;

    srslte_debug_handle_crash(argc, argv);

    parse_args(&prog_args, argc, argv);


    for (auto & d : data) {
        d = srslte_vec_u8_malloc(2000 * 8);
        if (!d) {
            ERROR("Allocationg data");
            go_exit = true;
        }
    }

    uint8_t mch_table[10];
    bzero(&mch_table[0], sizeof(uint8_t) * 10);
    if (prog_args.mbsfn_area_id > -1) {
        generate_mcch_table(mch_table, prog_args.mbsfn_sf_mask);
    }

    float search_cell_cfo = 0;

    int decimate = 0;
    if (prog_args.decimate) {
        if (prog_args.decimate > 4 || prog_args.decimate < 0) {
            printf("Invalid decimation factor, setting to 1 \n");
        } else {
            decimate = prog_args.decimate;
        }
    }
    if (srslte_ue_sync_init_multi_decim(&ue_sync,
                                        cell.nof_prb,
                                        cell.id == 1000,
                                        srslte_rf_recv_wrapper,
                                        prog_args.rf_nof_rx_ant,
                                        (void*)&rf,
                                        decimate)) {
        ERROR("Error initiating ue_sync\n");
        exit(-1);
    }
    if (srslte_ue_sync_set_cell(&ue_sync, cell)) {
        ERROR("Error initiating ue_sync\n");
        exit(-1);
    }


    uint32_t max_num_samples = 3 * SRSLTE_SF_LEN_PRB(cell.nof_prb); /// Length in complex samples
    for (int i = 0; i < prog_args.rf_nof_rx_ant; i++) {
        sf_buffer[i] = srslte_vec_cf_malloc(max_num_samples);
    }
    srslte_ue_mib_t ue_mib;
    if (srslte_ue_mib_init(&ue_mib, sf_buffer[0], cell.nof_prb)) {
        ERROR("Error initaiting UE MIB decoder\n");
        exit(-1);
    }
    if (srslte_ue_mib_set_cell(&ue_mib, cell)) {
        ERROR("Error initaiting UE MIB decoder\n");
        exit(-1);
    }

    if (srslte_ue_dl_init(&ue_dl, sf_buffer, cell.nof_prb, prog_args.rf_nof_rx_ant)) {
        ERROR("Error initiating UE downlink processing module\n");
        exit(-1);
    }
    if (srslte_ue_dl_set_cell(&ue_dl, cell)) {
        ERROR("Error initiating UE downlink processing module\n");
        exit(-1);
    }

    // Disable CP based CFO estimation during find
    ue_sync.cfo_current_value       = search_cell_cfo / 15000;
    ue_sync.cfo_is_copied           = true;
    ue_sync.cfo_correct_enable_find = true;
    srslte_sync_set_cfo_cp_enable(&ue_sync.sfind, false, 0);

    ZERO_OBJECT(ue_dl_cfg);
    ZERO_OBJECT(dl_sf);
    ZERO_OBJECT(pdsch_cfg);

    if (cell.frame_type == SRSLTE_TDD && prog_args.tdd_special_sf >= 0 && prog_args.sf_config >= 0) {
        dl_sf.tdd_config.ss_config  = prog_args.tdd_special_sf;
        dl_sf.tdd_config.sf_config  = prog_args.sf_config;
        dl_sf.tdd_config.configured = true;
    }

    srslte_chest_dl_cfg_t chest_pdsch_cfg = {};
    chest_pdsch_cfg.cfo_estimate_enable   = prog_args.enable_cfo_ref;
    chest_pdsch_cfg.cfo_estimate_sf_mask  = 1023;
    chest_pdsch_cfg.estimator_alg         = srslte_chest_dl_str2estimator_alg(prog_args.estimator_alg);
    chest_pdsch_cfg.sync_error_enable     = true;

    // Special configuration for MBSFN channel estimation
    srslte_chest_dl_cfg_t chest_mbsfn_cfg = {};
    chest_mbsfn_cfg.filter_type           = SRSLTE_CHEST_FILTER_TRIANGLE;
    chest_mbsfn_cfg.filter_coef[0]        = 0.1;
    chest_mbsfn_cfg.estimator_alg         = SRSLTE_ESTIMATOR_ALG_INTERPOLATE;
    chest_mbsfn_cfg.noise_alg             = SRSLTE_NOISE_ALG_PSS;

    // Allocate softbuffer buffers
    srslte_softbuffer_rx_t rx_softbuffers[SRSLTE_MAX_CODEWORDS];
    for (uint32_t i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
        pdsch_cfg.softbuffers.rx[i] = &rx_softbuffers[i];
        srslte_softbuffer_rx_init(pdsch_cfg.softbuffers.rx[i], cell.nof_prb);
    }

    pdsch_cfg.rnti = prog_args.rnti;

    /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
    srslte_ue_dl_set_rnti(&ue_dl, prog_args.rnti);

    /* Configure MBSFN area id and non-MBSFN Region */
    if (prog_args.mbsfn_area_id > -1) {
        srslte_ue_dl_set_mbsfn_area_id(&ue_dl, prog_args.mbsfn_area_id);
        srslte_ue_dl_set_non_mbsfn_region(&ue_dl, prog_args.non_mbsfn_region);
    }

    if (!prog_args.input_file_name) {
        srslte_rf_start_rx_stream(&rf, false);
    }

    if (prog_args.rf_gain < 0 && !prog_args.input_file_name) {
        srslte_rf_info_t* rf_info = srslte_rf_get_info(&rf);
        srslte_ue_sync_start_agc(&ue_sync,
                                 srslte_rf_set_rx_gain_th_wrapper_,
                                 rf_info->min_rx_gain,
                                 rf_info->max_rx_gain,
                                 cell_detect_config.init_agc);
    }


    ue_sync.cfo_correct_enable_track = !prog_args.disable_cfo;

    srslte_pbch_decode_reset(&ue_mib.pbch);

    INFO("\nEntering main loop...\n\n");

    // Variables for measurements
    uint32_t nframes = 0;
    float    rsrp0 = 0.0, rsrp1 = 0.0, rsrq = 0.0, snr = 0.0, enodebrate = 0.0, uerate = 0.0, procrate = 0.0,
            sinr[SRSLTE_MAX_LAYERS][SRSLTE_MAX_CODEBOOKS] = {}, sync_err[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS] = {};
    bool decode_pdsch = false;

    for (auto & s : sinr) {
        srslte_vec_f_zero(s, SRSLTE_MAX_CODEBOOKS);
    }

    /* Main loop */
    uint64_t sf_cnt          = 0;
    uint32_t sfn             = 0;
    uint32_t last_decoded_tm = 0;

    while (!go_exit && (sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1)) {
        char input[128];
        PRINT_LINE_INIT();

        fd_set set;
        FD_ZERO(&set);
        FD_SET(0, &set);

        struct timeval to;
        to.tv_sec  = 0;
        to.tv_usec = 0;

        /* Set default verbose level */
        srslte_verbose = prog_args.verbose;
        int n          = select(1, &set, NULL, NULL, &to);
        if (n == 1) {
            /* If a new line is detected set verbose level to Debug */
            if (fgets(input, sizeof(input), stdin)) {
                srslte_verbose = SRSLTE_VERBOSE_DEBUG;
                pkt_errors     = 0;
                pkt_total      = 0;
                nof_detected   = 0;
                nof_trials     = 0;
            }
        }

        cf_t* buffers[SRSLTE_MAX_CHANNELS] = {};
        for (int p = 0; p < SRSLTE_MAX_PORTS; p++) {
            buffers[p] = sf_buffer[p];
        }
        ret = srslte_ue_sync_zerocopy(&ue_sync, buffers, max_num_samples);
        if (ret < 0) {
            ERROR("Error calling srslte_ue_sync_work()\n");
        }

#ifdef CORRECT_SAMPLE_OFFSET
        float sample_offset =
        (float)srslte_ue_sync_get_last_sample_offset(&ue_sync) + srslte_ue_sync_get_sfo(&ue_sync) / 1000;
    srslte_ue_dl_set_sample_offset(&ue_dl, sample_offset);
#endif

        /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
        if (ret == 1) {

            bool           acks[SRSLTE_MAX_CODEWORDS] = {false};
            struct timeval t[3];

            uint32_t sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);

            switch (state) {
                case DECODE_MIB:
                    if (sf_idx == 0) {
                        uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
                        int     sfn_offset;
                        n = srslte_ue_mib_decode(&ue_mib, bch_payload, NULL, &sfn_offset);
                        if (n < 0) {
                            ERROR("Error decoding UE MIB\n");
                            exit(-1);
                        } else if (n == SRSLTE_UE_MIB_FOUND) {
                            srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
                            srslte_cell_fprint(stdout, &cell, sfn);
                            printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
                            sfn   = (sfn + sfn_offset) % 1024;
                            state = DECODE_PDSCH;
                        }
                    }
                    break;
                case DECODE_PDSCH:

                    if (prog_args.rnti != SRSLTE_SIRNTI) {
                        decode_pdsch = true;
                        if (srslte_sfidx_tdd_type(dl_sf.tdd_config, sf_idx) == SRSLTE_TDD_SF_U) {
                            decode_pdsch = false;
                        }
                    } else {
                        /* We are looking for SIB1 Blocks, search only in appropiate places */
                        if ((sf_idx == 5 && (sfn % 2) == 0) || mch_table[sf_idx] == 1) {
                            decode_pdsch = true;
                        } else {
                            decode_pdsch = false;
                        }
                    }

                    uint32_t tti = sfn * 10 + sf_idx;

                    gettimeofday(&t[1], NULL);
                    if (decode_pdsch) {

//            n = srslte_ue_dl_decode_pdsch(&ue_dl, &ue_dl_cfg, &pdsch_cfg, *data);
                        srslte_sf_t sf_type;
                        if (mch_table[sf_idx] == 0 || prog_args.mbsfn_area_id < 0) { // Not an MBSFN subframe
                            sf_type = SRSLTE_SF_NORM;

                            // Set PDSCH channel estimation
                            ue_dl_cfg.chest_cfg = chest_pdsch_cfg;
                        } else {
                            sf_type = SRSLTE_SF_MBSFN;

                            // Set MBSFN channel estimation
                            ue_dl_cfg.chest_cfg = chest_mbsfn_cfg;
                        }

                        n = 0;
                        for (uint32_t tm = 0; tm < 4 && !n; tm++) {
                            dl_sf.tti                             = tti;
                            dl_sf.sf_type                         = sf_type;
                            ue_dl_cfg.cfg.tm                      = (srslte_tm_t)tm;
                            ue_dl_cfg.cfg.pdsch.use_tbs_index_alt = prog_args.enable_256qam;

                            if ((ue_dl_cfg.cfg.tm == SRSLTE_TM1 && cell.nof_ports == 1) ||
                                (ue_dl_cfg.cfg.tm > SRSLTE_TM1 && cell.nof_ports > 1)) {
                                n = srslte_ue_dl_find_and_decode(&ue_dl, &dl_sf, &ue_dl_cfg, &pdsch_cfg, data, acks);
                                printf("%d", pdsch_cfg.grant.nof_tb);
                                srslte_vec_fprint_byte(stdout, *data, pdsch_cfg.grant.tb[0].tbs/8);
                                if (n > 0) {
                                    nof_detected++;
                                    last_decoded_tm = tm;
                                    for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
                                        if (pdsch_cfg.grant.tb[tb].enabled) {
                                            if (!acks[tb]) {
                                                if (sf_type == SRSLTE_SF_NORM) {
                                                    pkt_errors++;
                                                } else {
                                                    pmch_pkt_errors++;
                                                }
                                            }
                                            if (sf_type == SRSLTE_SF_NORM) {
                                                pkt_total++;
                                            } else {
                                                pmch_pkt_total++;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        // Feed-back ue_sync with chest_dl CFO estimation
                        if (sf_idx == 5 && prog_args.enable_cfo_ref) {
                            srslte_ue_sync_set_cfo_ref(&ue_sync, ue_dl.chest_res.cfo);
                        }

                        gettimeofday(&t[2], NULL);
                        get_time_interval(t);

                        if (n > 0) {
//#ifdef PRINT_CHANGE_SCHEDULING
//                            if (pdsch_cfg.dci.cw[0].mcs_idx != old_dl_dci.cw[0].mcs_idx ||
//                  memcmp(&pdsch_cfg.dci.type0_alloc, &old_dl_dci.type0_alloc, sizeof(srslte_ra_type0_t)) ||
//                  memcmp(&pdsch_cfg.dci.type1_alloc, &old_dl_dci.type1_alloc, sizeof(srslte_ra_type1_t)) ||
//                  memcmp(&pdsch_cfg.dci.type2_alloc, &old_dl_dci.type2_alloc, sizeof(srslte_ra_type2_t))) {
//                old_dl_dci = pdsch_cfg.dci;
//                fflush(stdout);
//                printf("DCI %s\n", srslte_dci_format_string(pdsch_cfg.dci.dci_format));
//                srslte_ra_pdsch_fprint(stdout, &old_dl_dci, cell.nof_prb);
//              }
//#endif
                        }

                        nof_trials++;

                        uint32_t enb_bits = ((pdsch_cfg.grant.tb[0].enabled ? pdsch_cfg.grant.tb[0].tbs : 0) +
                                             (pdsch_cfg.grant.tb[1].enabled ? pdsch_cfg.grant.tb[1].tbs : 0));
                        uint32_t ue_bits  = ((acks[0] ? pdsch_cfg.grant.tb[0].tbs : 0) + (acks[1] ? pdsch_cfg.grant.tb[1].tbs : 0));
                        rsrq              = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_dbm, rsrq, 0.1f);
                        rsrp0             = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_port_dbm[0], rsrp0, 0.05f);
                        rsrp1             = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_port_dbm[1], rsrp1, 0.05f);
                        snr               = SRSLTE_VEC_EMA(ue_dl.chest_res.snr_db, snr, 0.05f);
                        enodebrate        = SRSLTE_VEC_EMA(enb_bits / 1000.0f, enodebrate, 0.05f);
                        uerate            = SRSLTE_VEC_EMA(ue_bits / 1000.0f, uerate, 0.001f);
                        if (chest_pdsch_cfg.sync_error_enable) {
                            for (uint32_t i = 0; i < cell.nof_ports; i++) {
                                for (uint32_t j = 0; j < prog_args.rf_nof_rx_ant; j++) {
                                    sync_err[i][j] = SRSLTE_VEC_EMA(ue_dl.chest.sync_err[i][j], sync_err[i][j], 0.001f);
                                    if (!isnormal(sync_err[i][j])) {
                                        sync_err[i][j] = 0.0f;
                                    }
                                }
                            }
                        }
                        float elapsed = (float)t[0].tv_usec + t[0].tv_sec * 1.0e+6f;
                        if (elapsed != 0.0f) {
                            procrate = SRSLTE_VEC_EMA(ue_bits / elapsed, procrate, 0.01f);
                        }

                        nframes++;
                        if (isnan(rsrq)) {
                            rsrq = 0;
                        }
                        if (isnan(snr)) {
                            snr = 0;
                        }
                        if (isnan(rsrp0)) {
                            rsrp0 = 0;
                        }
                        if (isnan(rsrp1)) {
                            rsrp1 = 0;
                        }
                    }

                    // Plot and Printf
                    if (sf_idx == 5) {
                        float gain = prog_args.rf_gain;
                        if (gain < 0) {
                            gain = srslte_convert_power_to_dB(srslte_agc_get_gain(&ue_sync.agc));
                        }

                        /* Print transmission scheme */

                        /* Print basic Parameters */
                        PRINT_LINE("          CFO: %+7.2f Hz", srslte_ue_sync_get_cfo(&ue_sync));
                        PRINT_LINE("         RSRP: %+5.1f dBm | %+5.1f dBm", rsrp0, rsrp1);
                        PRINT_LINE("          SNR: %+5.1f dB", snr);
                        PRINT_LINE("           TM: %d", last_decoded_tm + 1);
                        PRINT_LINE(
                                "           Rb: %6.2f / %6.2f / %6.2f Mbps (net/maximum/processing)", uerate, enodebrate, procrate);
                        PRINT_LINE("   PDCCH-Miss: %5.2f%%", 100 * (1 - (float)nof_detected / nof_trials));
                        PRINT_LINE("   PDSCH-BLER: %5.2f%%", (float)100 * pkt_errors / pkt_total);

                        if (prog_args.mbsfn_area_id > -1) {
                            PRINT_LINE("   PMCH-BLER: %5.2f%%", (float)100 * pkt_errors / pmch_pkt_total);
                        }

                        PRINT_LINE("         TB 0: mcs=%d; tbs=%d", pdsch_cfg.grant.tb[0].mcs_idx, pdsch_cfg.grant.tb[0].tbs);
                        PRINT_LINE("         TB 1: mcs=%d; tbs=%d", pdsch_cfg.grant.tb[1].mcs_idx, pdsch_cfg.grant.tb[1].tbs);

                        /* MIMO: if tx and rx antennas are bigger than 1 */
                        if (cell.nof_ports > 1 && ue_dl.pdsch.nof_rx_antennas > 1) {
                            uint32_t ri = 0;
                            float    cn = 0;
                            /* Compute condition number */
                            if (srslte_ue_dl_select_ri(&ue_dl, &ri, &cn)) {
                                /* Condition number calculation is not supported for the number of tx & rx antennas*/
                                PRINT_LINE("            κ: NA");
                            } else {
                                /* Print condition number */
                                PRINT_LINE("            κ: %.1f dB, RI=%d (Condition number, 0 dB => Best)", cn, ri);
                            }
                            PRINT_LINE("");
                        }
                        if (chest_pdsch_cfg.sync_error_enable) {
                            for (uint32_t i = 0; i < cell.nof_ports; i++) {
                                for (uint32_t j = 0; j < prog_args.rf_nof_rx_ant; j++) {
                                    PRINT_LINE("sync_err[%d][%d]=%f", i, j, sync_err[i][j]);
                                }
                            }
                        }
                        PRINT_LINE("Press enter maximum printing debug log of 1 subframe.");
                        PRINT_LINE("");
                        PRINT_LINE_RESET_CURSOR();
                    }
                    break;
            }
            if (sf_idx == 9) {
                sfn++;
                if (sfn == 1024) {
                    sfn = 0;
                    PRINT_LINE_ADVANCE_CURSOR();
                    pkt_errors      = 0;
                    pkt_total       = 0;
                    pmch_pkt_errors = 0;
                    pmch_pkt_total  = 0;
                }
            }
        } else if (ret == 0) {
            printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r",
                   srslte_sync_get_peak_value(&ue_sync.sfind),
                   ue_sync.frame_total_cnt,
                   ue_sync.state);
        }

        sf_cnt++;
    } // Main loop
    srslte_ue_dl_free(&ue_dl);
    srslte_ue_sync_free(&ue_sync);
    for (auto & i : data) {
        if (i) {
            free(i);
        }
    }
    for (int i = 0; i < prog_args.rf_nof_rx_ant; i++) {
        if (sf_buffer[i]) {
            free(sf_buffer[i]);
        }
    }

#ifndef DISABLE_RF
    if (!prog_args.input_file_name) {
        srslte_ue_mib_free(&ue_mib);
        srslte_rf_close(&rf);
    }
#endif

    printf("\nBye\n");
    exit(0);
    return 0;
}