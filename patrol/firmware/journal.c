#include "journal.h"

_Static_assert(sizeof(Event) * PAGE_SLOTS == 1024, "TM4C flash page");
static uint32_t checksum(const Event *e) {
    uint32_t crc = ~0u;
    uint32_t words[] = {e->id, e->ms, e->code};
    for (unsigned i = 0; i < 3; ++i) {
        crc ^= words[i];
        for (unsigned bit = 0; bit < 32; ++bit)
            crc = (crc >> 1) ^ ((0u - (crc & 1)) & 0xedb88320u);
    }
    return ~crc;
}
static bool valid(const Event *e) {
    return e->id != 0 && e->id != UINT32_MAX && e->check == checksum(e);
}

void journal_open(Journal *j, const volatile Event *slots) {
    *j = (Journal){.slots = slots};
    for (unsigned i = 0; i < LOG_SLOTS; i++) {
        Event e = slots[i];
        if (valid(&e) && (!j->newest || (int32_t)(e.id - j->newest) > 0)) {
            j->newest = e.id;
            j->next = (i + 1) % LOG_SLOTS;
        }
    }
}

void journal_append(Journal *j, uint32_t ms, unsigned code) {
    if (j->failed) return;
    unsigned slot = j->next;
    /* Skip torn records. At a page boundary the previous newest record
       is on the other page, so interruption during erase preserves it. */
    while (slot % PAGE_SLOTS) {
        Event e = j->slots[slot];
        if ((e.id & e.ms & e.code & e.check) == UINT32_MAX) break;
        slot = (slot + 1) % LOG_SLOTS;
    }
    uint32_t id = j->newest + 1;
    if (!id || id == UINT32_MAX) id = 1;
    Event e = {id, ms, code, 0};
    e.check = checksum(&e);
    if ((slot % PAGE_SLOTS == 0 && !log_erase_page(slot)) || !log_program(slot, &e)) {
        j->failed = true;
        return;
    }
    j->newest = id;
    j->next = (slot + 1) % LOG_SLOTS;
}

Event journal_after(const Journal *j, uint32_t cursor) {
    Event found = {0};
    if (!j || j->newest == cursor) return found;
    for (unsigned i = 0; i < LOG_SLOTS; ++i) {
        Event e = j->slots[i];
        if (valid(&e) && (!cursor || (int32_t)(e.id - cursor) > 0) &&
            (!found.id || (int32_t)(e.id - found.id) < 0))
            found = e;
    }
    return found;
}
