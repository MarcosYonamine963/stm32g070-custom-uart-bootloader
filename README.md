## Custom UART Bootloader for STM32G070 microcontrollers

Project created on STM Cube IDE 1.14.0

## The propose

The initial idea of this project was to partition the 128kB flash of the STM32G070 as shown below (each page has 2kB). But this repo contains only the bootloader, since this can be used for many other projects.

<p align="center">
  <img width='250' src="https://github.com/MarcosYonamine963/stm32g070-uart-bootloader/assets/92953755/f0c8311e-15ee-4dac-af17-76fd4bedf791" />
</p>

[Image](https://www.figma.com/file/XMHS7XS03KrujKNYSkjsni/Bootloader-Flash-Memory-map?type=design&node-id=0%3A1&mode=design&t=wlQlL7hvSKh4BXtO-1) done on Figma.

The flash configuration has four partitions: Bootloader, APP1, APP2 and EEPROM. The APP1 is the main Application (not implemented on this repo). When running APP1, the idea is to receive a new firmware data and store it at APP2. After successfully receive a new firmware, a variable at EEPROM shoud be updated. 

On system reset (or startup), the bootloader is the first partition to run. It'll check the EEPROM variable to verify if there is a new firmware available at APP2. If there is, then, the bootloader copies all contents of APP2 to APP1, reset the EEPROM variable, and start the APP1.




## Configuring Project's Addresses

All start address must be a start PAGE address. On G070, the page has 2kB lenght.

### Bootloader's Apllication address

The Bootloader's application must be written whith the start page at default address (0x08000000).

But, we must change it's length.

On the linker file (`STM32G70KBTX_FLASH.ld`), change the flash memory length:



``` diff
/* Memories definition */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 36K
-  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 128K
+  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 20K
}
```



Some definitions, and values regarding addresses that you must pay attention:

At `main.c` file, the APP1 and APP2 start address (they must be a start page address):

```
#define DEFAULT_APP1_START_ADDRESS 0x08005000
#define DEFAULT_APP2_START_ADDRESS 0x08012000
```

At `write_data_to_flash_app` function, there is a routine to erase flash pages. It is configured to erase APP1.

```
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError;

EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
EraseInitStruct.Page        = 10;
EraseInitStruct.NbPages     = 26;

ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
```

### Main Apllication address

The main application must be a separate STM project.

The main application address must be configured to APP1's start address (0x08005000).

On the linker file of the main application project (`STM32G70KBTX_FLASH.ld`), change the flash memory origin and length:

``` diff
/* Memories definition */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 36K
-  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 128K
+  FLASH    (rx)    : ORIGIN = 0x8005000,   LENGTH = 52K
}
```



Similarly to bootloader, to erase the APP2, you can use the following routine:

```
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError;

EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
EraseInitStruct.Page          = 36; // Start page of APP2
EraseInitStruct.NbPages       = 26; // Quant pages of APP2

ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
```


On `system_stm32g0xx.c` file of the main application project, the start address must be configured using `VECT_TAB_OFFSET`, by uncommenting `#define USER_VECT_TAB_ADDRESS`:


```diff
/************************* Miscellaneous Configuration ************************/
/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
- /* #define USER_VECT_TAB_ADDRESS */
+ #define USER_VECT_TAB_ADDRESS

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
- #define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
+ #define VECT_TAB_OFFSET         0x00005000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */


#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/
```









