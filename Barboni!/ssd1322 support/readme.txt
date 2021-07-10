/home/pi/linux/drivers/staging/fbtft:
1. KCONFIG: inserire il contenuto del file kconfig.merge
2. copiare in questa directory il file fb_ssd1322.c

menuconfig (kernel build):
[D]evice Drivers --> [S]taging Drivers --> [S]upport for small TFT LCD display modules -->
<M> SSD1322 driver


File makefile:
aggiungere
obj-$(CONFIG_FB_TFT_SSD1322)     += fb_ssd1322.o

file modules.order:
aggiungere
drivers/staging/fbtft/fb_ssd1322.ko

ricompilare lo kernel
