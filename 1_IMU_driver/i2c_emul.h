#ifndef I2C_EMUL_H_
#define I2C_EMUL_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    bool create_cmd_pipe();
    void unlink_cmd_pipe();

#ifdef __cplusplus
}
#endif

#endif /* I2C_EMUL_H_ End of header guard */