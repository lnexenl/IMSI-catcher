//
// Created by lnex on 2021/4/9.
//
#include <cassert>
#include <cmath>
#include <pthread.h>
#include <semaphore.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "srslte/common/crash_handler.h"
#include "srslte/common/gen_mch_tables.h"
#include "srslte/common/buffer_pool.h"
#include "srslte/asn1/rrc_asn1.h"
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
    int nof_subframes;
    int cpu_affinity;
    bool disable_plots;
    bool disable_plots_except_constellation;
    bool disable_cfo;
    uint32_t time_offset;
    int force_N_id_2;
    uint16_t rnti;
    char *input_file_name;
    int file_offset_time;
    float file_offset_freq;
    uint32_t file_nof_prb;
    uint32_t file_nof_ports;
    uint32_t file_cell_id;
    bool enable_cfo_ref;
    char *estimator_alg;
    char *rf_dev;
    char *rf_args;
    uint32_t rf_nof_rx_ant;
    double rf_freq;
    float rf_gain;
    int net_port;
    char *net_address;
    int net_port_signal;
    char *net_address_signal;
    int decimate;
    int32_t mbsfn_area_id;
    uint8_t non_mbsfn_region;
    uint8_t mbsfn_sf_mask;
    int tdd_special_sf;
    int sf_config;
    int verbose;
    bool enable_256qam;
} prog_args_t;

void args_default(prog_args_t *args) {
  args->disable_plots = false;
  args->disable_plots_except_constellation = false;
  args->nof_subframes = -1;
  args->rnti = SRSLTE_SIRNTI;
  args->force_N_id_2 = -1; // Pick the best
  args->tdd_special_sf = -1;
  args->sf_config = -1;
  args->input_file_name = nullptr;
  args->disable_cfo = false;
  args->time_offset = 0;
  args->file_nof_prb = 25;
  args->file_nof_ports = 1;
  args->file_cell_id = 0;
  args->file_offset_time = 0;
  args->file_offset_freq = 0;
  args->rf_dev = (char *) "";
  args->rf_args = (char *) "";
  args->rf_freq = -1.0;
  args->rf_nof_rx_ant = 1;
  args->enable_cfo_ref = false;
  args->estimator_alg = (char *) "interpolate";
  args->enable_256qam = false;
#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1.0;
#else
  args->rf_gain = 50.0;
#endif
  args->net_port = -1;
  args->net_address = (char *) "127.0.0.1";
  args->net_port_signal = -1;
  args->net_address_signal = (char *) "127.0.0.1";
  args->decimate = 0;
  args->cpu_affinity = -1;
  args->mbsfn_area_id = -1;
  args->non_mbsfn_region = 2;
  args->mbsfn_sf_mask = 32;
}

void usage(prog_args_t *args, char *prog) {
  printf("Usage: %s [adgpPoOcildFRDnrMNvTG] -f rx_frequency (in Hz) | -i input_file\n", prog);
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
  printf("\t-y set the cpu affinity mask [Default %d] \n  ", args->cpu_affinity);
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-M MBSFN area id [Default %d]\n", args->mbsfn_area_id);
  printf("\t-N Non-MBSFN region [Default %d]\n", args->non_mbsfn_region);
  printf("\t-q Enable/Disable 256QAM modulation (default %s)\n", args->enable_256qam ? "enabled" : "disabled");
  printf("\t-v [set srslte_verbose to debug, default none]\n");
}

void parse_args(prog_args_t *args, int argc, char **argv) {
  int opt;
  args_default(args);

  while ((opt = getopt(argc, argv, "adAogliIpPcOCtdDFRqnvrfuUsSZyMNBTG")) != -1) {
    switch (opt) {
      case 'i':
        args->input_file_name = argv[optind];
        break;
      case 'p':
        args->file_nof_prb = (uint32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'P':
        args->file_nof_ports = (uint32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'o':
        args->file_offset_freq = strtof(argv[optind], nullptr);
        break;
      case 'O':
        args->file_offset_time = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'c':
        args->file_cell_id = (uint32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'I':
        args->rf_dev = argv[optind];
        break;
      case 'a':
        args->rf_args = argv[optind];
        break;
      case 'A':
        args->rf_nof_rx_ant = (uint32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'g':
        args->rf_gain = strtof(argv[optind], nullptr);
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
        args->time_offset = (uint32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'f':
        args->rf_freq = strtod(argv[optind], nullptr);
        break;
      case 'T':
        args->tdd_special_sf = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'G':
        args->sf_config = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'n':
        args->nof_subframes = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'r':
        args->rnti = strtol(argv[optind], nullptr, 16);
        break;
      case 'l':
        args->force_N_id_2 = (int) strtol(argv[optind], nullptr, 10);
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
        args->decimate = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'y':
        args->cpu_affinity = (int) strtol(argv[optind], nullptr, 10);
        break;
      case 'M':
        args->mbsfn_area_id = (int32_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'N':
        args->non_mbsfn_region = (uint8_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'B':
        args->mbsfn_sf_mask = (uint8_t) strtol(argv[optind], nullptr, 10);
        break;
      case 'q':
        args->enable_256qam ^= true;
        break;
      default:
        usage(args, argv[0]);
        exit(-1);
    }
  }
  if (args->rf_freq < 0 && args->input_file_name == nullptr) {
    usage(args, argv[0]);
    exit(-1);
  }
}
//srsue::rrc*                      rrc_ptr;

uint8_t *data[SRSLTE_MAX_CODEWORDS];

bool go_exit = false;

void sig_int_handler(int signo) {
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

cf_t *sf_buffer[SRSLTE_MAX_PORTS] = {nullptr};

int srslte_rf_recv_wrapper(void *h, cf_t *data_[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  void *ptr[SRSLTE_MAX_PORTS];
  for (int i = 0; i < SRSLTE_MAX_PORTS; i++) {
    ptr[i] = data_[i];
  }
  return srslte_rf_recv_with_time_multi((srslte_rf_t *) h, ptr, nsamples, true, nullptr, nullptr);
}

static SRSLTE_AGC_CALLBACK(srslte_rf_set_rx_gain_th_wrapper_) {
  srslte_rf_set_rx_gain_th((srslte_rf_t *) h, gain_db);
}

extern float mean_exec_time;

enum receiver_state {
    DECODE_MIB, DECODE_SIB1, DECODE_OTHER_SIB
} state;

srslte_cell_t cell;
srslte_ue_dl_t ue_dl;
srslte_ue_dl_cfg_t ue_dl_cfg;
srslte_dl_sf_cfg_t dl_sf;
srslte_pdsch_cfg_t pdsch_cfg;
srslte_ue_sync_t ue_sync;
prog_args_t prog_args;

uint32_t pkt_errors = 0, pkt_total = 0, nof_detected = 0, pmch_pkt_errors = 0, pmch_pkt_total = 0, nof_trials = 0;


int main(int argc, char **argv) {
  int ret;

  srslte_rf_t rf;

  srslte_debug_handle_crash(argc, argv);

  parse_args(&prog_args, argc, argv);


  for (auto &d : data) {
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
  if (prog_args.cpu_affinity > -1) {

    cpu_set_t cpuset;
    pthread_t thread;

    thread = pthread_self();
    for (int i = 0; i < 8; i++) {
      if (((prog_args.cpu_affinity >> i) & 0x01) == 1) {
        printf("Setting pdsch_ue with affinity to core %d\n", i);
        CPU_SET((size_t) i, &cpuset);
      }
      if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        ERROR("Error setting main thread affinity to %d \n", prog_args.cpu_affinity);
        exit(-1);
      }
    }
  }
  float search_cell_cfo = 0;
  if (!prog_args.input_file_name) {

    printf("Opening RF device with %d RX antennas...\n", prog_args.rf_nof_rx_ant);
    if (srslte_rf_open_devname(&rf, prog_args.rf_dev, prog_args.rf_args, prog_args.rf_nof_rx_ant)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    /* Set receiver gain */
    if (prog_args.rf_gain > 0) {
      srslte_rf_set_rx_gain(&rf, prog_args.rf_gain);
    } else {
      printf("Starting AGC thread...\n");
      if (srslte_rf_start_gain_thread(&rf, false)) {
        ERROR("Error opening rf\n");
        exit(-1);
      }
      srslte_rf_set_rx_gain(&rf, srslte_rf_get_rx_gain(&rf));
      cell_detect_config.init_agc = srslte_rf_get_rx_gain(&rf);
    }

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigprocmask(SIG_UNBLOCK, &sigset, nullptr);
    signal(SIGINT, sig_int_handler);

    /* set receiver frequency */
    printf("Tunning receiver to %.3f MHz\n", (prog_args.rf_freq + prog_args.file_offset_freq) / 1000000);
    srslte_rf_set_rx_freq(&rf, prog_args.rf_nof_rx_ant, prog_args.rf_freq + prog_args.file_offset_freq);

    uint32_t ntrial = 0;
    do {
      ret = rf_search_and_decode_mib(
              &rf, prog_args.rf_nof_rx_ant, &cell_detect_config, prog_args.force_N_id_2, &cell, &search_cell_cfo);
      if (ret < 0) {
        ERROR("Error searching for cell\n");
        exit(-1);
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
      }
    } while (ret == 0 && !go_exit);

    if (go_exit) {
      srslte_rf_close(&rf);
      exit(0);
    }

    /* set sampling frequency */
    int srate = srslte_sampling_freq_hz(cell.nof_prb);
    if (srate != -1) {
      printf("Setting sampling rate %.2f MHz\n", (float) srate / 1000000);
      float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
      if (srate_rf != srate) {
        ERROR("Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      ERROR("Invalid number of PRB %d\n", cell.nof_prb);
      exit(-1);
    }

    INFO("Stopping RF and flushing buffer...\r");
  }

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
                                      (void *) &rf,
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
  ue_sync.cfo_current_value = search_cell_cfo / 15000;
  ue_sync.cfo_is_copied = true;
  ue_sync.cfo_correct_enable_find = true;
  srslte_sync_set_cfo_cp_enable(&ue_sync.sfind, false, 0);

  ZERO_OBJECT(ue_dl_cfg);
  ZERO_OBJECT(dl_sf);
  ZERO_OBJECT(pdsch_cfg);

  if (cell.frame_type == SRSLTE_TDD && prog_args.tdd_special_sf >= 0 && prog_args.sf_config >= 0) {
    dl_sf.tdd_config.ss_config = prog_args.tdd_special_sf;
    dl_sf.tdd_config.sf_config = prog_args.sf_config;
    dl_sf.tdd_config.configured = true;
  }

  srslte_chest_dl_cfg_t chest_pdsch_cfg = {};
  chest_pdsch_cfg.cfo_estimate_enable = prog_args.enable_cfo_ref;
  chest_pdsch_cfg.cfo_estimate_sf_mask = 1023;
  chest_pdsch_cfg.estimator_alg = srslte_chest_dl_str2estimator_alg(prog_args.estimator_alg);
  chest_pdsch_cfg.sync_error_enable = true;

  // Special configuration for MBSFN channel estimation
  srslte_chest_dl_cfg_t chest_mbsfn_cfg = {};
  chest_mbsfn_cfg.filter_type = SRSLTE_CHEST_FILTER_TRIANGLE;
  chest_mbsfn_cfg.filter_coef[0] = 0.1;
  chest_mbsfn_cfg.estimator_alg = SRSLTE_ESTIMATOR_ALG_INTERPOLATE;
  chest_mbsfn_cfg.noise_alg = SRSLTE_NOISE_ALG_PSS;

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
    srslte_rf_info_t *rf_info = srslte_rf_get_info(&rf);
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
  float rsrp0 = 0.0, rsrp1 = 0.0, rsrq = 0.0, snr = 0.0, enodebrate = 0.0, uerate = 0.0, procrate = 0.0,
          sinr[SRSLTE_MAX_LAYERS][SRSLTE_MAX_CODEBOOKS] = {}, sync_err[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS] = {};

  std::vector<int> si_periodicity;
  std::vector<std::vector<int>> sibs;
  asn1::rrc::bcch_dl_sch_msg_s sib_info;
  asn1::json_writer j;
  bool decode_sib = false;
  bool decoded_sib1 = false;
  int win_len = 0;
  int n_decode = 0;
  for (auto &s : sinr) {
    srslte_vec_f_zero(s, SRSLTE_MAX_CODEBOOKS);
  }

  /* Main loop */
  uint64_t sf_cnt = 0;
  uint32_t sfn = 0;
//  uint32_t last_decoded_tm = 0;

  while (!go_exit && (sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1)) {
    char input[128];
    PRINT_LINE_INIT();

    fd_set set;
    FD_ZERO(&set);
    FD_SET(0, &set);

    struct timeval to;
    to.tv_sec = 0;
    to.tv_usec = 0;

    /* Set default verbose level */
    srslte_verbose = prog_args.verbose;
    int n = select(1, &set, nullptr, nullptr, &to);
    if (n == 1) {
      /* If a new line is detected set verbose level to Debug */
      if (fgets(input, sizeof(input), stdin)) {
        srslte_verbose = SRSLTE_VERBOSE_DEBUG;
        pkt_errors = 0;
        pkt_total = 0;
        nof_detected = 0;
        nof_trials = 0;
      }
    }

    cf_t *buffers[SRSLTE_MAX_CHANNELS] = {};
    for (int p = 0; p < SRSLTE_MAX_PORTS; p++) {
      buffers[p] = sf_buffer[p];
    }
    ret = srslte_ue_sync_zerocopy(&ue_sync, buffers, max_num_samples);
    if (ret < 0) {
      ERROR("Error calling srslte_ue_sync_work()\n");
    }

    /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
    if (ret == 1) {

      bool acks[SRSLTE_MAX_CODEWORDS] = {false};
      struct timeval t[3];

      uint32_t sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);

      switch (state) {
        case DECODE_MIB: {
          if (sf_idx == 0) {
            uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
            int sfn_offset;
            n = srslte_ue_mib_decode(&ue_mib, bch_payload, nullptr, &sfn_offset);
            if (n < 0) {
              ERROR("Error decoding UE MIB\n");
              exit(-1);
            } else if (n == SRSLTE_UE_MIB_FOUND) {
              srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
              srslte_cell_fprint(stdout, &cell, sfn);
              printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              sfn = (sfn + sfn_offset) % 1024;
              state = DECODE_SIB1;
            }
          }
          break;
        }
        case DECODE_SIB1: {

          if (prog_args.rnti != SRSLTE_SIRNTI) {
            decode_sib = true;
            if (srslte_sfidx_tdd_type(dl_sf.tdd_config, sf_idx) == SRSLTE_TDD_SF_U) {
              decode_sib = false;
            }
          } else {
            /* We are looking for SIB1 Blocks, search only in appropiate places */
//            printf("sf_idx=%d, sfn=%d, mch_table[sf_idx]=%d, n_decode=%d\n",sf_idx, sfn, mch_table[sf_idx], n_decode);
            if (((sf_idx == 5 && (sfn % 2) == 0) || mch_table[sf_idx] == 1) && n_decode == 0 && (!decoded_sib1)) {
              printf("trying to decode sib1\n");
              decode_sib = true;
            }
            else if(n_decode > 0) {
//              printf("trying to decode other sibs\n");
              decode_sib = false;
                int x = (n_decode - 1) * win_len;
                int a = x % 10;
                int sfn_e = floor(x/10);
//                printf("x=%d, a=%dm sfn_e=%d\n", x,a,sfn_e);
                if (sf_idx == a && (sfn % si_periodicity[n_decode-1]) == sfn_e) {
                  printf("trying to decode other sibs, subframe=%d, sf_idx=%d, sfn=%d, si_periodicity=%d, sfn_e=%d\n", a, sf_idx, sfn, si_periodicity[n_decode-1], sfn_e);
                  decode_sib = true;

                }
            }
            else {
              decode_sib = false;
            }
          }

          uint32_t tti = sfn * 10 + sf_idx;

          gettimeofday(&t[1], nullptr);
          if (decode_sib) {
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
              dl_sf.tti = tti;
              dl_sf.sf_type = sf_type;
              ue_dl_cfg.cfg.tm = (srslte_tm_t) tm;
              ue_dl_cfg.cfg.pdsch.use_tbs_index_alt = prog_args.enable_256qam;

              if ((ue_dl_cfg.cfg.tm == SRSLTE_TM1 && cell.nof_ports == 1) ||
                  (ue_dl_cfg.cfg.tm > SRSLTE_TM1 && cell.nof_ports > 1)) {
                n = srslte_ue_dl_find_and_decode(&ue_dl, &dl_sf, &ue_dl_cfg, &pdsch_cfg, data, acks);
              }
              printf("%d\n",n);
            }
            // Feed-back ue_sync with chest_dl CFO estimation
            if (sf_idx == 5 && prog_args.enable_cfo_ref) {
              srslte_ue_sync_set_cfo_ref(&ue_sync, ue_dl.chest_res.cfo);
            }

            if (n > 0) {
              asn1::cbit_ref bref{*data, uint32_t(pdsch_cfg.grant.tb[0].tbs / 8)};

              if (n_decode == 0) {
                // extracting SIB sched info from SIB1
                asn1::rrc::bcch_dl_sch_msg_s sib1_msg;
                printf("%d %d\n", pdsch_cfg.grant.tb[0].tbs, pdsch_cfg.grant.tb[1].tbs);
                if (sib1_msg.unpack(bref) == asn1::SRSASN_SUCCESS) {
                  srslte_vec_fprint_byte(stdout, *data, pdsch_cfg.grant.tb[0].tbs / 8);
                  printf("Decoding SIB successful.\n");
                  sib1_msg.to_json(j);
                  std::cout<<j.to_string()<<std::endl;
                  asn1::rrc::sib_type1_s &sib1 = sib1_msg.msg.c1().sib_type1();
                  win_len = sib1.si_win_len.to_number();
                  printf("window-length:%d\n", win_len);
                  for (auto &i : sib1.sched_info_list) {
                    si_periodicity.push_back(i.si_periodicity.to_number());
                    printf("%s\n", i.si_periodicity.to_string().c_str());
                    std::vector<int> sibs_;
                    if (&i == sib1.sched_info_list.begin()) // SIB 2
                      sibs_.push_back(2);
                    for (auto &j : i.sib_map_info) {
                      sibs_.push_back(j.to_number());
                      printf("%s\n", j.to_string().c_str());
                    }
                    sibs.push_back(sibs_);
                  }
                  n_decode++;
                  decoded_sib1 = true;
                  printf("\n");
                } else printf("Decoding SIB1 failed.\n");
              } else if (n_decode > 0) {
                std::cout << "n_decode: " << n_decode << std::endl;
                if (sib_info.unpack(bref) == asn1::SRSASN_SUCCESS) {
                  std::cout << "Decoding SIB ";
                  for (auto &it:sibs[n_decode - 1]) {
                    std::cout << it << " ";
                  }
                  std::cout << "successfully." << std::endl;
                  srslte_vec_fprint_byte(stdout, *data, pdsch_cfg.grant.tb[0].tbs / 8);
                  n_decode++;
                  if (n_decode == si_periodicity.size() + 1) {
                    sib_info.to_json(j);
                    std::cout<<j.to_string()<<std::endl;
                    exit(0);
                  }
                } else {
                  std::cout << "Decoding SIB ";
                  for (auto &it:sibs[n_decode - 1]) {
                    std::cout << it << " ";
                  }
                  std::cout << "failed." << std::endl;
                }
              }
            }

            gettimeofday(&t[2], nullptr);
            get_time_interval(t);

            nof_trials++;
          }
          break;
        }
      }
      if (sf_idx == 9) {
        sfn++;
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
  for (auto &i : data) {
    if (i) {
      free(i);
    }
  }
  for (int i = 0; i < prog_args.rf_nof_rx_ant; i++) {
    if (sf_buffer[i]) {
      free(sf_buffer[i]);
    }
  }

  if (!prog_args.input_file_name) {
    srslte_ue_mib_free(&ue_mib);
    srslte_rf_close(&rf);
  }
  printf("\nBye\n");
  exit(0);
  return 0;
}