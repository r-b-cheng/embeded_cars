#include "clib_cmd.h"
#include "clib_port.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/* 原厂 6 条 + 常用别名 + 导航扩展 */
const clib_cmd_entry_t CLIB_CMD_TABLE[] = {
    { "ONA",       CLIB_CMD_FORWARD,     "原厂：前进" },
    { "ONB",       CLIB_CMD_BACKWARD,    "原厂：后退" },
    { "ONC",       CLIB_CMD_LEFT,        "原厂：左转" },
    { "OND",       CLIB_CMD_RIGHT,       "原厂：右转" },
    { "ONE",       CLIB_CMD_STOP,        "原厂：停止" },
    { "ONF",       CLIB_CMD_STOP,        "原厂：停止" },
    { "FWD",       CLIB_CMD_FORWARD,     "通用别名" },
    { "BACK",      CLIB_CMD_BACKWARD,    "通用别名" },
    { "LEFT",      CLIB_CMD_LEFT,        "通用别名" },
    { "RIGHT",     CLIB_CMD_RIGHT,       "通用别名" },
    { "SPINL",     CLIB_CMD_SPIN_LEFT,   "原地左旋" },
    { "SPINR",     CLIB_CMD_SPIN_RIGHT,  "原地右旋" },
    { "STOP",      CLIB_CMD_STOP,        "通用别名" },
    { "MODE:TRACK",CLIB_CMD_MODE_TRACK,  "切循迹模式" },
    { "MODE:AVOID",CLIB_CMD_MODE_AVOID,  "切避障模式" },
    { "MODE:REMOTE",CLIB_CMD_MODE_REMOTE,"切遥控模式" },
    { "MODE:NAV",  CLIB_CMD_MODE_NAV,    "切点对点导航模式" },
    { "?",         CLIB_CMD_QUERY,       "查询状态" }
};
const uint8_t CLIB_CMD_TABLE_LEN =
    (uint8_t)(sizeof(CLIB_CMD_TABLE) / sizeof(CLIB_CMD_TABLE[0]));

/* 不依赖 locale 的 ASCII 大写化 */
static char upper_ascii(char c)
{
    unsigned char u = (unsigned char)c;
    if (u >= 'a' && u <= 'z') {
        return (char)(u - ('a' - 'A'));
    }
    return (char)u;
}

static bool ieq(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (upper_ascii(*a) != upper_ascii(*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return (*a == '\0' && *b == '\0');
}

static bool iprefix(const char *s, const char *prefix)
{
    while (*prefix != '\0') {
        if (*s == '\0' || upper_ascii(*s) != upper_ascii(*prefix)) {
            return false;
        }
        ++s;
        ++prefix;
    }
    return true;
}

static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        ++s;
    }
    return s;
}

/* 轻量字符串拼接：刻意不用 snprintf / strtol 之外的 libc 重活，
   避免把 printf 格式化引擎（约 3~8KB Flash）链进 256KB Flash 的 MCU 工程 */
static size_t append_str(char *dst, size_t pos, size_t cap, const char *s)
{
    while (*s != '\0' && pos + 1u < cap) {
        dst[pos++] = *s++;
    }
    dst[pos] = '\0';
    return pos;
}

static size_t append_int(char *dst, size_t pos, size_t cap, int v)
{
    char tmp[12];
    size_t n = 0u;
    uint32_t u;

    if (v < 0) {
        pos = append_str(dst, pos, cap, "-");
        u = (uint32_t)(-(int64_t)v);     /* 用 64 位避免 INT_MIN 取负溢出 */
    } else {
        u = (uint32_t)v;
    }
    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u != 0u && n < sizeof(tmp));
    while (n > 0u && pos + 1u < cap) {
        dst[pos++] = tmp[--n];
    }
    dst[pos] = '\0';
    return pos;
}

clib_cmd_t clib_cmd_parse(const char *frame)
{
    const char *s;
    char  token[24];
    uint8_t i;
    uint8_t n = 0u;

    if (frame == NULL) {
        return CLIB_CMD_NONE;
    }
    s = skip_ws(frame);
    if (*s == '\0') {
        return CLIB_CMD_NONE;
    }
    /* 截取到第一个分隔符，去掉 \r\n */
    while (*s != '\0' && *s != ' ' && *s != '\t' &&
           *s != '\r' && *s != '\n' && *s != ',') {
        if (n < (uint8_t)(sizeof(token) - 1u)) {
            token[n++] = *s;
        }
        ++s;
    }
    token[n] = '\0';

    for (i = 0u; i < CLIB_CMD_TABLE_LEN; ++i) {
        if (ieq(token, CLIB_CMD_TABLE[i].text)) {
            return CLIB_CMD_TABLE[i].cmd;
        }
    }
    if (iprefix(token, "GOTO")) {
        return CLIB_CMD_GOTO;
    }
    return CLIB_CMD_UNKNOWN;
}

static const char *const s_cmd_names[] = {
    "NONE", "FORWARD", "BACKWARD", "LEFT", "RIGHT",
    "SPIN_LEFT", "SPIN_RIGHT", "STOP",
    "MODE_TRACK", "MODE_AVOID", "MODE_REMOTE", "MODE_NAV",
    "GOTO", "QUERY", "UNKNOWN"
};

const char *clib_cmd_name(clib_cmd_t cmd)
{
    if ((int)cmd < 0 || (int)cmd >= (int)(sizeof(s_cmd_names) / sizeof(s_cmd_names[0]))) {
        return "INVALID";
    }
    return s_cmd_names[(int)cmd];
}

clib_act_t clib_cmd_to_act(clib_cmd_t cmd)
{
    switch (cmd) {
    case CLIB_CMD_FORWARD:    return CLIB_ACT_FORWARD;
    case CLIB_CMD_BACKWARD:   return CLIB_ACT_BACKWARD;
    case CLIB_CMD_LEFT:       return CLIB_ACT_LEFT;
    case CLIB_CMD_RIGHT:      return CLIB_ACT_RIGHT;
    case CLIB_CMD_SPIN_LEFT:  return CLIB_ACT_SPIN_LEFT;
    case CLIB_CMD_SPIN_RIGHT: return CLIB_ACT_SPIN_RIGHT;
    case CLIB_CMD_STOP:       return CLIB_ACT_STOP;
    default:                  return CLIB_ACT_STOP;
    }
}

int clib_cmd_send(uint8_t uart_id, const char *msg)
{
    size_t len;
    int sent;

    if (msg == NULL) {
        return CLIB_ERR_NULL;
    }
    if (!clib_is_ready()) {
        return CLIB_ERR_NOT_INIT;
    }
    len = strlen(msg);
    if (len == 0u) {
        return CLIB_OK;
    }
    if (len > 65535u) {
        return CLIB_ERR_OVERFLOW;
    }
    sent = clib_uart_write(uart_id, (const uint8_t *)msg, (uint16_t)len);
    if (sent < 0) {
        return sent;    /* 保留底层错误码 */
    }
    return (sent == (int)len) ? CLIB_OK : CLIB_ERR_IO;
}

int clib_cmd_send_ack(uint8_t uart_id, clib_cmd_t cmd, int result)
{
    char buf[64];
    size_t pos = 0u;

    pos = append_str(buf, pos, sizeof(buf), "ACK ");
    pos = append_str(buf, pos, sizeof(buf), clib_cmd_name(cmd));
    pos = append_str(buf, pos, sizeof(buf), " ");
    pos = append_int(buf, pos, sizeof(buf), result);
    pos = append_str(buf, pos, sizeof(buf), "\r\n");
    return clib_cmd_send(uart_id, buf);
}

bool clib_cmd_parse_goto(const char *frame, int16_t *x, int16_t *y)
{
    const char *s;
    char *end = NULL;
    long vx;
    long vy;

    if (frame == NULL || x == NULL || y == NULL) {
        return false;
    }
    s = skip_ws(frame);
    if (!iprefix(s, "GOTO")) {
        return false;
    }
    s += 4;
    s = skip_ws(s);

    errno = 0;
    vx = strtol(s, &end, 10);
    if (end == s || errno == ERANGE) {
        return false;
    }
    s = skip_ws(end);
    if (*s == ',') {
        ++s;
    }
    s = skip_ws(s);
    errno = 0;
    vy = strtol(s, &end, 10);
    if (end == s || errno == ERANGE) {
        return false;
    }
    /* 尾部只允许空白，"GOTO 12,-3junk" 必须拒绝 */
    s = skip_ws(end);
    if (*s != '\0') {
        return false;
    }
    if (vx < -32768L || vx > 32767L || vy < -32768L || vy > 32767L) {
        return false;
    }
    *x = (int16_t)vx;
    *y = (int16_t)vy;
    return true;
}
