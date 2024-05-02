#define MCP4725_PD_NONE 0
#define MCP4725_PD_1k 1
#define MCP4725_PD_100k 2
#define MCP4725_PD_500k 3

void mcp4725_open();
void mcp4725_close();
void mcp4725_set_dac(int value, int powerdown, int set_eeprom);
void mcp4725_get_status(int *value, int *eeprom_value, int *pd, int *busy);
void mcp4725_disable_dac(int diable);
