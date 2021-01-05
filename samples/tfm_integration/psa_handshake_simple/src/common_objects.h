#include <zephyr.h>
#include <kernel.h>


extern struct k_mbox ca_root_mailbox;
extern struct k_mbox end_node_mailbox;

extern void end_node_entry(void *, void *, void *);
extern void provisioning_entry(void *, void *, void *);
extern void root_ca_entry(void *, void *, void *);

extern struct k_thread end_node;
extern struct k_thread provisioning;
extern struct k_thread root_ca;

extern k_tid_t end_node_tid;
extern k_tid_t provisioning_tid;
extern k_tid_t root_ca_tid;

typedef enum {
    NACK,
    ACK,
    ARE_YOU_UP,
} msg_types;