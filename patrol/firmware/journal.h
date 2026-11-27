#ifndef PATROL_JOURNAL_H
#define PATROL_JOURNAL_H
#include <stdint.h>
#include <stdbool.h>

typedef struct { uint32_t id, ms, code, check; } Event;
enum { PAGE_SLOTS = 64, LOG_SLOTS = 2 * PAGE_SLOTS };
enum { EV_BOOT = 1, EV_DISCONNECT, EV_LEASE, EV_BUTTON, EV_FEEDBACK, EV_BLOCKED, EV_OUTPUT };
typedef struct {
    const volatile Event *slots;
    uint32_t newest;
    unsigned next;
    bool failed;
} Journal;
void journal_open(Journal *j, const volatile Event *slots);
void journal_append(Journal *j, uint32_t ms, unsigned code);
Event journal_after(const Journal *j, uint32_t cursor);
bool log_erase_page(unsigned slot);
bool log_program(unsigned slot, const Event *event);
#endif
