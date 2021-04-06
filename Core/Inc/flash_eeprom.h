
#ifndef __flash_eeprom_h_
#define __flash_eeprom_h_

void eeprom_write_to_flash(void);
void eeprom_read_from_flash(void);
uint8_t *eeprom_get_buff_address(void);



#endif