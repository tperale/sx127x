#include "lora.h"
#include <cc.h>

int t_sym(radio_sf sf, radio_bw bw)
{
  return LORA_T_SYM(sf, bw);
}

int t_preamble(lora_radio_t *radio)
{
  return LORA_T_PREAMBLE(radio->sf, radio->bw, radio->prlen);
}

int payload_sym_nb(lora_radio_t *radio, size_t len)
{
  return LORA_SYM_NB((radio->sf), radio->crc, radio->implicit_header, (radio->cr), len);
}

int t_payload(lora_radio_t *radio, size_t len)
{
  return LORA_SYM_NB(radio->sf, radio->crc, radio->implicit_header, radio->cr, len) * LORA_T_SYM(radio->sf, radio->bw);
}

int t_packet(lora_radio_t *radio, size_t len)
{
  return LORA_T_PACKET(radio->sf, radio->bw, radio->crc, radio->implicit_header, radio->cr, radio->prlen, len);
}
