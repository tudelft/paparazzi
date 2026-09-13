/**
 * @file chdk_pipe.h
 * @brief Lifecycle for CATIA's CHDKPTP shell-script camera backend.
 * @details The external controller stays alive between shots. Its stdout is a prompt
 * protocol parsed by chdk_pipe.c, while CATIA owns process cleanup and retry policy.
 */

/** @brief Start the external CHDK controller and configure the connected camera.
 * @return 0 after the controller reaches record mode, otherwise -1. */
int chdk_pipe_init(void);
/** @brief Request one CHDK remote capture.
 * @param filename Receives the controller-reported image path or an empty string on failure.
 * @warning The caller must provide at least MAX_FILENAME bytes. */
void chdk_pipe_shoot(char *filename);
/** @brief Kill and reap the controller process and close its pipes. */
void chdk_pipe_deinit(void);

