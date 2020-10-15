#include "lora.h"
#include <cc.h>
#include <math.h>

int t_sym(radio_sf sf, radio_bw bw)
{
  return (1 << sf) * 1000 / bw;
}

int t_preamble(lora_radio_t *radio)
{
  return (radio->prlen + 4) * t_sym(radio->sf, radio->bw) + (t_sym(radio->sf, radio->bw) / 4);
}

int payload_sym_nb(lora_radio_t *radio, size_t len)
{
  return 8 + MAX(
      ceil((float) ( (8 * len) - (4 * radio->sf) + 28 + (16 * radio->crc) - (20 * radio->implicit_header) )
       / (4 * (radio->sf))
      ) * (radio->cr + 4),
      0
  );
}

int t_payload(lora_radio_t *radio, size_t len)
{
  return payload_sym_nb(radio, len) * t_sym(radio->sf, radio->bw);
}

int t_packet(lora_radio_t *radio, size_t len)
{
  return t_preamble(radio) + t_payload(radio, len);
}
