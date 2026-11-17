#include "clib_common.h"

const char *clib_version(void)
{
    return CLIB_VERSION_STRING;
}

const char *clib_err_str(clib_err_t e)
{
    switch (e) {
    case CLIB_OK:            return "ok";
    case CLIB_ERR_NOT_INIT:  return "not initialized";
    case CLIB_ERR_NULL:      return "null pointer";
    case CLIB_ERR_PARAM:     return "invalid parameter";
    case CLIB_ERR_NO_PORT:   return "port callback missing";
    case CLIB_ERR_TIMEOUT:   return "timeout";
    case CLIB_ERR_CHECKSUM:  return "checksum mismatch";
    case CLIB_ERR_OVERFLOW:  return "buffer overflow";
    case CLIB_ERR_IO:        return "io error";
    case CLIB_ERR_BUSY:      return "busy";
    case CLIB_ERR_FORMAT:    return "bad frame format";
    default:                 return "unknown error";
    }
}

const char *clib_act_name(clib_act_t a)
{
    switch (a) {
    case CLIB_ACT_STOP:       return "STOP";
    case CLIB_ACT_FORWARD:    return "FORWARD";
    case CLIB_ACT_BACKWARD:   return "BACKWARD";
    case CLIB_ACT_LEFT:       return "LEFT";
    case CLIB_ACT_RIGHT:      return "RIGHT";
    case CLIB_ACT_SPIN_LEFT:  return "SPIN_LEFT";
    case CLIB_ACT_SPIN_RIGHT: return "SPIN_RIGHT";
    case CLIB_ACT_BRAKE:      return "BRAKE";
    default:                  return "UNKNOWN";
    }
}
