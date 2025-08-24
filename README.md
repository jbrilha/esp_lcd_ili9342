# ESP LCD ILI9342

[![Component Registry](https://components.espressif.com/components/jbrilha/esp_lcd_ili9342/badge.svg)](https://components.espressif.com/components/jbrilha/esp_lcd_ili9342)

This component prvides an implementation of the ILI9342(C) LCD driver using the `esp_lcd` component API, adapted from Espressif's own [ILI9341 component](https://components.espressif.com/components/espressif/esp_lcd_ili9341).

| LCD controller | Communication interface | Component name  |                                    Link to datasheet                                    |
| :------------: | :---------------------: | :-------------: | :-------------------------------------------------------------------------------------: |
|    ILI9342     |    SPI (half-duplex)*   | esp_lcd_ili9342 | [Specification](https://www.displayfuture.com/Display/datasheet/controller/ILI9342.pdf) |

## Usage in a project

1. As an ESP-IDF component, via `idf.py add-dependency`:

```bash
idf.py add-dependency "jbrilha/esp_lcd_ili9342^1.0.0"
```

2. Or including in it `idf_component.yml`

   - As a dependency:

   ```bash
   dependencies:
     jbrilha/esp_lcd_ili9342: "1.0.0"
   ```

   - As a direct repository

   ```bash
   dependencies:
     esp_lcd_ili9342:
       git: https://github.com/jbrilha/esp_lcd_ili9342.git
   ```

## *Important note regarding SPI

This display driver uses half-duplex communication, so instead of the usual MOSI/MISO separation, it uses a single SDA line.

This means that SPI must be configured as such:

```c
void init_lcd_spi() {
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_CLK_PIN,
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = -1, // set to -1
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
        .flags = SPICOMMON_BUSFLAG_SCLK |
                 // SPICOMMON_BUSFLAG_MISO | // ommit this flag
                 SPICOMMON_BUSFLAG_MOSI |
                 SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM};
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}
```

## Supported devices

Only tested on an M5Stack Core Basic (v2.7) as it is its embedded display, but should work with other ESP32 variants as well

## Examples

An example is present in the `examples` directory, using LVGL
