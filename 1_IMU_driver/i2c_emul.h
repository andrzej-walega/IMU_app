#ifndef I2C_EMUL_H_
#define I2C_EMUL_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// pipe for sending commands to IMU
#define PIPE_CMD_NAME "/tmp/imu_cmd_pipe"
// pipe for sending responses from IMU
#define PIPE_RESP_NAME "/tmp/imu_resp_pipe"
    
#define I2C_EMUL_SHOW_COMMUNICATION 0 /* 1 => show*/
#define IMU_SIMUL_SHOW_COMMUNICATION 1 /* 1 => show*/
#define GET_RESPONSE_TIMEOUT 500 /* [ms] */

#define SLAVE_OK "S,OK"
#define SLAVE_ERR "S,ERR"

    void ignore_sigpipe();
    void create_pipes();
    void close_pipes();

#ifdef __cplusplus
}
#endif

#endif /* I2C_EMUL_H_ End of header guard */