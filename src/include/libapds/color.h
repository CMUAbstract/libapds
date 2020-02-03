#ifndef _COLOR_H_
#define _COLOR_H_

#define COLOR_MASK 0x1

void apds_color_init();
void apds_read_color(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void apds_dummy_read_color(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void apds_color_disable();
void apds_color_reenable();
float apds_read_r(void);
#endif
